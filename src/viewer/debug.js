import * as THREE from 'three';

const TYPE_COLORS = {
  'tread':      0x00ff44,
  'riser':      0xff4444,
  'wall-left':  0x44ddff,
  'wall-right': 0xffdd44,
  'floor':      0xaaff44,
  'ceiling':    0xffffff,
  'wall-end':   0xff44ff,
};

export function buildQuadDebug(collisionQuads) {
  const group = new THREE.Group();

  for (const quad of collisionQuads) {
    const color = TYPE_COLORS[quad.type] ?? 0x888888;

    // Wireframe outline
    const geo = new THREE.BufferGeometry();
    geo.setAttribute(
      'position',
      new THREE.BufferAttribute(new Float32Array(quad.vertices.flat()), 3)
    );
    group.add(new THREE.LineLoop(geo, new THREE.LineBasicMaterial({ color })));

    // Normal arrow from quad center
    const sum = quad.vertices.reduce(
      (acc, v) => [acc[0] + v[0], acc[1] + v[1], acc[2] + v[2]],
      [0, 0, 0]
    );
    const n = quad.vertices.length;
    const origin = new THREE.Vector3(sum[0] / n, sum[1] / n, sum[2] / n);
    const dir = new THREE.Vector3(...quad.normal).normalize();
    group.add(new THREE.ArrowHelper(dir, origin, 0.3, color));
  }

  return group;
}
