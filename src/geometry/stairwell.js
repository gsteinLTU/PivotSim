import * as THREE from 'three';

/**
 * Each collision quad: { type, segment, vertices: [[x,y,z], ...], normal: [nx,ny,nz] }
 * segment: 'stair' | 'bottom-hall' | 'top-hall'
 * Vertices are in counter-clockwise order when viewed from the normal direction.
 */

export function buildStairwell(params) {
  const group = new THREE.Group();
  const collisionQuads = [];

  const { numSteps, risePerStep, runPerStep, stairWidth } = params;
  const halfW = stairWidth / 2;

  // Materials
  const treadMat = new THREE.MeshStandardMaterial({ color: 0x8899aa, side: THREE.DoubleSide });
  const riserMat = new THREE.MeshStandardMaterial({ color: 0x667788, side: THREE.DoubleSide });

  for (let i = 0; i < numSteps; i++) {
    const y = (i + 1) * risePerStep;
    const z = i * runPerStep;
    const zNext = (i + 1) * runPerStep;

    // Tread (horizontal surface of the step)
    const treadGeo = new THREE.PlaneGeometry(stairWidth, runPerStep);
    const treadMesh = new THREE.Mesh(treadGeo, treadMat);
    treadMesh.rotation.x = -Math.PI / 2;
    treadMesh.position.set(0, y, z + runPerStep / 2);
    group.add(treadMesh);

    collisionQuads.push({
      type: 'tread',
      segment: 'stair',
      vertices: [
        [-halfW, y, z],
        [halfW, y, z],
        [halfW, y, zNext],
        [-halfW, y, zNext],
      ],
      normal: [0, 1, 0],
    });

    // Riser (vertical face of the step)
    const riserGeo = new THREE.PlaneGeometry(stairWidth, risePerStep);
    const riserMesh = new THREE.Mesh(riserGeo, riserMat);
    riserMesh.position.set(0, y - risePerStep / 2, z);
    group.add(riserMesh);

    collisionQuads.push({
      type: 'riser',
      segment: 'stair',
      vertices: [
        [-halfW, y - risePerStep, z],
        [halfW, y - risePerStep, z],
        [halfW, y, z],
        [-halfW, y, z],
      ],
      normal: [0, 0, -1],
    });
  }

  // --- Walls along the stair flight ---
  const totalRise = numSteps * risePerStep;
  const totalRun = numSteps * runPerStep;
  const wallHeight = params.ceilingHeight;

  const wallMat = new THREE.MeshStandardMaterial({
    color: 0xccccdd,
    side: THREE.DoubleSide,
    transparent: true,
    opacity: 0.3,
    depthWrite: false, // Prevent z-fighting with treads/risers
  });

  // Left wall
  addStairWall(group, collisionQuads, wallMat, {
    x: -halfW,
    numSteps,
    risePerStep,
    runPerStep,
    wallHeight,
    normalX: 1,
    type: 'wall-left',
  });

  // Right wall
  addStairWall(group, collisionQuads, wallMat, {
    x: halfW,
    numSteps,
    risePerStep,
    runPerStep,
    wallHeight,
    normalX: -1,
    type: 'wall-right',
  });

  // --- Ceiling ---
  if (params.slopedCeiling) {
    const ceilY0 = params.ceilingHeight;
    const ceilY1 = params.ceilingHeight + totalRise;
    const ceilMat = new THREE.MeshStandardMaterial({
      color: 0xeeeeff,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.2,
    });

    const ceilGeo = new THREE.BufferGeometry();
    const ceilVerts = new Float32Array([
      -halfW, ceilY0, 0,
      halfW, ceilY0, 0,
      halfW, ceilY1, totalRun,
      -halfW, ceilY1, totalRun,
    ]);
    const ceilIndices = [0, 1, 2, 0, 2, 3];
    ceilGeo.setAttribute('position', new THREE.BufferAttribute(ceilVerts, 3));
    ceilGeo.setIndex(ceilIndices);
    ceilGeo.computeVertexNormals();
    const ceilMesh = new THREE.Mesh(ceilGeo, ceilMat);
    ceilMesh.userData.isSurface = true;
    group.add(ceilMesh);

    collisionQuads.push({
      type: 'ceiling',
      segment: 'stair',
      vertices: [
        [-halfW, ceilY0, 0],
        [halfW, ceilY0, 0],
        [halfW, ceilY1, totalRun],
        [-halfW, ceilY1, totalRun],
      ],
      // Normal points from ceiling surface into the stairwell (downward + toward stair base)
      // Computed as cross product of width edge × slope edge, then normalized
      normal: new THREE.Vector3(0, -totalRun, totalRise).normalize().toArray(),
    });
  }

  // --- Hallways ---
  buildHallway(group, collisionQuads, params, 'bottom');
  buildHallway(group, collisionQuads, params, 'top');

  return { group, collisionQuads };
}

function addStairWall(group, quads, material, opts) {
  const { x, numSteps, risePerStep, runPerStep, wallHeight, normalX, type } = opts;

  // Simplified: one quad per step segment, from step tread to ceiling
  for (let i = 0; i < numSteps; i++) {
    const y0 = i * risePerStep;
    const y1 = (i + 1) * risePerStep;
    const z0 = i * runPerStep;
    const z1 = (i + 1) * runPerStep;
    const top0 = y0 + wallHeight;
    const top1 = y1 + wallHeight;

    const geo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      x, y0, z0,
      x, y1, z1,
      x, top1, z1,
      x, top0, z0,
    ]);
    geo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geo.setIndex([0, 1, 2, 0, 2, 3]);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, material);
    mesh.userData.isSurface = true;
    group.add(mesh);

    quads.push({
      type,
      segment: 'stair',
      vertices: [
        [x, y0, z0],
        [x, y1, z1],
        [x, top1, z1],
        [x, top0, z0],
      ],
      normal: [normalX, 0, 0],
    });
  }
}

function buildHallway(group, quads, params, position) {
  const {
    numSteps, risePerStep, runPerStep, stairWidth, ceilingHeight, hallwayLength,
  } = params;
  const totalRise = numSteps * risePerStep;
  const totalRun = numSteps * runPerStep;

  const hallwayWidth = position === 'bottom'
    ? params.bottomHallwayWidth
    : params.topHallwayWidth;
  const turnDeg = position === 'bottom'
    ? params.bottomHallwayTurn
    : params.topHallwayTurn;

  const hallHalfW = hallwayWidth / 2;
  const stairHalfW = stairWidth / 2;

  const segment = position === 'bottom' ? 'bottom-hall' : 'top-hall';

  // Create a sub-group for the hallway, then rotate it into position
  const hallGroup = new THREE.Group();
  const hallQuads = [];

  const floorMat = new THREE.MeshStandardMaterial({ color: 0x99aa88, side: THREE.DoubleSide });
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0xccccdd, side: THREE.DoubleSide, transparent: true, opacity: 0.3, 
    depthWrite: false,
  });
  const ceilMat = new THREE.MeshStandardMaterial({
    color: 0xeeeeff, side: THREE.DoubleSide, transparent: true, opacity: 0.2,
    depthWrite: false,
  });

  // Hallway extends in -Z direction (away from stairs) in local space
  const len = hallwayLength;

  // Floor
  const floorGeo = new THREE.PlaneGeometry(hallwayWidth, len);
  const floorMesh = new THREE.Mesh(floorGeo, floorMat);
  floorMesh.rotation.x = -Math.PI / 2;
  floorMesh.position.set(0, 0, -len / 2);
  hallGroup.add(floorMesh);

  hallQuads.push({
    type: 'floor',
    segment,
    vertices: [
      [-hallHalfW, 0, 0],
      [hallHalfW, 0, 0],
      [hallHalfW, 0, -len],
      [-hallHalfW, 0, -len],
    ],
    normal: [0, 1, 0],
  });

  // Left wall
  const lwGeo = new THREE.PlaneGeometry(len, ceilingHeight);
  const lwMesh = new THREE.Mesh(lwGeo, wallMat);
  lwMesh.rotation.y = Math.PI / 2;
  lwMesh.position.set(-hallHalfW, ceilingHeight / 2, -len / 2);
  lwMesh.userData.isSurface = true;
  hallGroup.add(lwMesh);

  let leftStartZ = 0;
  if (turnDeg < 0) {
      leftStartZ = -stairWidth;
  }

  hallQuads.push({
    type: 'wall-left',
    segment,
    vertices: [
      [-hallHalfW, 0, leftStartZ],
      [-hallHalfW, 0, -len],
      [-hallHalfW, ceilingHeight, -len],
      [-hallHalfW, ceilingHeight, leftStartZ],
    ],
    normal: [1, 0, 0],
  });

  // Right wall
  const rwGeo = new THREE.PlaneGeometry(len, ceilingHeight);
  const rwMesh = new THREE.Mesh(rwGeo, wallMat);
  rwMesh.rotation.y = -Math.PI / 2;
  rwMesh.position.set(hallHalfW, ceilingHeight / 2, -len / 2);
  rwMesh.userData.isSurface = true;
  hallGroup.add(rwMesh);

  let rightStartZ = 0;
  if (turnDeg > 0) {
      rightStartZ = -stairWidth;
  }

  hallQuads.push({
    type: 'wall-right',
    segment,
    vertices: [
      [hallHalfW, 0, rightStartZ],
      [hallHalfW, 0, -len],
      [hallHalfW, ceilingHeight, -len],
      [hallHalfW, ceilingHeight, rightStartZ],
    ],
    normal: [-1, 0, 0],
  });

  // Ceiling
  const cGeo = new THREE.PlaneGeometry(hallwayWidth, len);
  const cMesh = new THREE.Mesh(cGeo, ceilMat);
  cMesh.rotation.x = Math.PI / 2;
  cMesh.position.set(0, ceilingHeight, -len / 2);
  cMesh.userData.isSurface = true;
  hallGroup.add(cMesh);

  hallQuads.push({
    type: 'ceiling',
    segment,
    vertices: [
      [-hallHalfW, ceilingHeight, 0],
      [hallHalfW, ceilingHeight, 0],
      [hallHalfW, ceilingHeight, -len],
      [-hallHalfW, ceilingHeight, -len],
    ],
    normal: [0, -1, 0],
  });

  // First end cap (prevents box from trying to travel through the void outside the stairwell)
  const ecGeo = new THREE.PlaneGeometry(hallwayWidth, ceilingHeight);
  const ecMesh = new THREE.Mesh(ecGeo, wallMat);
  ecMesh.position.set(0, ceilingHeight / 2, 0);
  ecMesh.userData.isSurface = true;
  hallGroup.add(ecMesh);
  
  hallQuads.push({
    type: 'wall-end',
    segment,
    vertices: [
      [-hallHalfW, 0, -len],
      [hallHalfW, 0, -len],
      [hallHalfW, ceilingHeight, -len],
      [-hallHalfW, ceilingHeight, -len],
    ],
    normal: [0, 0, 1],
  });

  // Second end cap (only when hallway is turned, fills gap where unrotated hallway meets stairwell)
  if (turnDeg !== 0) {
    const ecGeo = new THREE.PlaneGeometry(hallwayWidth, ceilingHeight);
    const ecMesh = new THREE.Mesh(ecGeo, wallMat);
    ecMesh.position.set(0, ceilingHeight / 2, -len);
    ecMesh.userData.isSurface = true;
    hallGroup.add(ecMesh);

    hallQuads.push({
      type: 'wall-end',
      segment,
      vertices: [
        [-hallHalfW, 0, 0],
        [hallHalfW, 0, 0],
        [hallHalfW, ceilingHeight, 0],
        [-hallHalfW, ceilingHeight, 0],
      ],
      normal: [0, 0, -1],
    });
  }

  // Position and rotate the hallway group
  const turnRad = (turnDeg * Math.PI) / 180;

  if (position === 'bottom') {
    hallGroup.rotation.y = -turnRad;
    hallGroup.position.set(0, 0, 0);
  } else {
    hallGroup.rotation.y = Math.PI - turnRad;
    hallGroup.position.set(0, totalRise, totalRun);
  }

  if (turnDeg !== 0) {
    console.log(`Applying ${turnDeg}° turn to ${position} hallway`);
    // Offset by half the stair width to align with the stairwell opening, and half the hallway width to align with the hallway centerline.
    hallGroup.position.x -= (stairHalfW) * (turnDeg > 0 ? 1 : -1) * (position === 'bottom' ? 1 : -1); // Shift opposite to turn direction to keep aligned with stairwell
    hallGroup.position.z += (hallHalfW) * (position === 'bottom' ? -1 : 1);
    console.log(`Hallway position after turn adjustment: (${hallGroup.position.x.toFixed(2)}, ${hallGroup.position.y.toFixed(2)}, ${hallGroup.position.z.toFixed(2)})`);
  }

  group.add(hallGroup);

  // Transform collision quads to world space
  const matrix = new THREE.Matrix4();
  matrix.makeRotationY(hallGroup.rotation.y);
  const pos = hallGroup.position;

  for (const quad of hallQuads) {
    const transformed = quad.vertices.map((v) => {
      const vec = new THREE.Vector3(v[0], v[1], v[2]);
      vec.applyMatrix4(matrix);
      vec.add(pos);
      return [vec.x, vec.y, vec.z];
    });
    // transformDirection uses the upper-left 3x3 only (ignores translation),
    // which is correct for direction vectors like normals.
    const nVec = new THREE.Vector3(...quad.normal);
    nVec.transformDirection(matrix);

    quads.push({
      type: quad.type,
      segment: quad.segment,
      vertices: transformed,
      normal: [nVec.x, nVec.y, nVec.z],
    });
  }
}
