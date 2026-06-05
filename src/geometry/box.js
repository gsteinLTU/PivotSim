import * as THREE from 'three';

/**
 * Creates a semi-transparent box mesh.
 * Local axes: X = width, Y = height, Z = length.
 * Default color green; update via mesh.material.color.setHex(...)
 */
export function createBoxMesh({ length, width, height }) {
  const geo = new THREE.BoxGeometry(width, height, length);
  const mat = new THREE.MeshStandardMaterial({
    color: 0x22ff88,
    transparent: true,
    opacity: 0.6,
    depthWrite: false,
  });
  return new THREE.Mesh(geo, mat);
}

/**
 * Sets box mesh position and rotation from a pose.
 * pose.yaw/pitch/roll must be in RADIANS.
 */
export function updateBoxMeshPose(mesh, { x, y, z, yaw, pitch, roll }) {
  mesh.position.set(x, y, z);
  mesh.rotation.set(pitch, yaw, roll, 'YXZ');
}

/**
 * Computes an OBB from pose (angles in RADIANS) and halfExtents.
 * halfExtents = [hWidth, hHeight, hLength] — use getHalfExtents() to produce this.
 * Returns { center: [x,y,z], axes: [[...],[...],[...]], halfExtents: [...] }
 */
export function computeOBB({ x, y, z, yaw, pitch, roll }, halfExtents) {
  const euler = new THREE.Euler(pitch, yaw, roll, 'YXZ');
  const mat = new THREE.Matrix4().makeRotationFromEuler(euler);
  const e = mat.elements; // column-major: col0=[0,1,2], col1=[4,5,6], col2=[8,9,10]
  return {
    center: [x, y, z],
    axes: [
      [e[0], e[1], e[2]],   // local X (width direction) in world space
      [e[4], e[5], e[6]],   // local Y (height direction) in world space
      [e[8], e[9], e[10]],  // local Z (length direction) in world space
    ],
    halfExtents,
  };
}

/**
 * Returns 8 corner positions as [x, y, z] arrays.
 */
export function getOBBCorners({ center, axes, halfExtents }) {
  const corners = [];
  for (let i = -1; i <= 1; i += 2) {
    for (let j = -1; j <= 1; j += 2) {
      for (let k = -1; k <= 1; k += 2) {
        corners.push([
          center[0] + i * halfExtents[0] * axes[0][0]
                    + j * halfExtents[1] * axes[1][0]
                    + k * halfExtents[2] * axes[2][0],
          center[1] + i * halfExtents[0] * axes[0][1]
                    + j * halfExtents[1] * axes[1][1]
                    + k * halfExtents[2] * axes[2][1],
          center[2] + i * halfExtents[0] * axes[0][2]
                    + j * halfExtents[1] * axes[1][2]
                    + k * halfExtents[2] * axes[2][2],
        ]);
      }
    }
  }
  return corners;
}

/**
 * Returns halfExtents array [hWidth, hHeight, hLength] from box dimensions.
 */
export function getHalfExtents({ length, width, height }) {
  return [width / 2, height / 2, length / 2];
}
