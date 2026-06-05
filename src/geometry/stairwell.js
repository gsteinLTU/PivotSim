import * as THREE from 'three';

/**
 * Each collision quad: { type, vertices: [[x,y,z], ...], normal: [nx,ny,nz] }
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
    const top = y1 + wallHeight;

    const geo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      x, y0, z0,
      x, y1, z1,
      x, top, z1,
      x, top, z0,
    ]);
    geo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    geo.setIndex([0, 1, 2, 0, 2, 3]);
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, material);
    mesh.userData.isSurface = true;
    group.add(mesh);

    quads.push({
      type,
      vertices: [
        [x, y0, z0],
        [x, y1, z1],
        [x, top, z1],
        [x, top, z0],
      ],
      normal: [normalX, 0, 0],
    });
  }
}

function buildHallway(group, quads, params, position) {
  const {
    numSteps, risePerStep, runPerStep, ceilingHeight, hallwayLength,
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

  // Create a sub-group for the hallway, then rotate it into position
  const hallGroup = new THREE.Group();
  const hallQuads = [];

  const floorMat = new THREE.MeshStandardMaterial({ color: 0x99aa88, side: THREE.DoubleSide });
  const wallMat = new THREE.MeshStandardMaterial({
    color: 0xccccdd, side: THREE.DoubleSide, transparent: true, opacity: 0.3,
  });
  const ceilMat = new THREE.MeshStandardMaterial({
    color: 0xeeeeff, side: THREE.DoubleSide, transparent: true, opacity: 0.2,
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

  hallQuads.push({
    type: 'wall-left',
    vertices: [
      [-hallHalfW, 0, 0],
      [-hallHalfW, 0, -len],
      [-hallHalfW, ceilingHeight, -len],
      [-hallHalfW, ceilingHeight, 0],
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

  hallQuads.push({
    type: 'wall-right',
    vertices: [
      [hallHalfW, 0, 0],
      [hallHalfW, 0, -len],
      [hallHalfW, ceilingHeight, -len],
      [hallHalfW, ceilingHeight, 0],
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
    vertices: [
      [-hallHalfW, ceilingHeight, 0],
      [hallHalfW, ceilingHeight, 0],
      [hallHalfW, ceilingHeight, -len],
      [-hallHalfW, ceilingHeight, -len],
    ],
    normal: [0, -1, 0],
  });

  // Position and rotate the hallway group
  const turnRad = (turnDeg * Math.PI) / 180;

  if (position === 'bottom') {
    hallGroup.rotation.y = -turnRad;
    hallGroup.position.set(0, 0, 0);
  } else {
    hallGroup.rotation.y = Math.PI - turnRad;
    hallGroup.position.set(0, totalRise, totalRun);
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
      vertices: transformed,
      normal: [nVec.x, nVec.y, nVec.z],
    });
  }
}
