function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(v) {
  const len = Math.sqrt(dot(v, v));
  if (len < 1e-10) return null;
  return [v[0] / len, v[1] / len, v[2] / len];
}

// Project OBB onto a unit axis, returns scalar interval { min, max }
function projectOBB(obb, axis) {
  const c = dot(obb.center, axis);
  const r = obb.halfExtents[0] * Math.abs(dot(obb.axes[0], axis))
           + obb.halfExtents[1] * Math.abs(dot(obb.axes[1], axis))
           + obb.halfExtents[2] * Math.abs(dot(obb.axes[2], axis));
  return { min: c - r, max: c + r };
}

/**
 * Returns gap on this axis:
 *   positive → shapes are separated by this amount
 *   negative → shapes overlap on this axis
 * Degenerate axes (zero length) return -Infinity so they never become the maxGap.
 */
function testAxis(obb, quadVerts, axis) {
  const n = normalize(axis);
  if (!n) return -Infinity;

  const obbRange = projectOBB(obb, n);
  const quadProjs = quadVerts.map((v) => dot(v, n));
  const quadRange = { min: Math.min(...quadProjs), max: Math.max(...quadProjs) };

  return Math.max(obbRange.min - quadRange.max, quadRange.min - obbRange.max);
}

/**
 * SAT test: OBB vs a single quad.
 * Returns { collides: boolean, clearance: number }
 *   collides=false → clearance = gap on the most-separating axis (approx lower bound on true distance)
 *   collides=true  → clearance = maxGap (≤ 0; magnitude is approximate penetration depth)
 */
export function testOBBvsQuad(obb, quad) {
  const verts = quad.vertices;

  // Two edge directions of the quad
  const e0 = normalize([
    verts[1][0] - verts[0][0],
    verts[1][1] - verts[0][1],
    verts[1][2] - verts[0][2],
  ]);
  const e1 = normalize([
    verts[3][0] - verts[0][0],
    verts[3][1] - verts[0][1],
    verts[3][2] - verts[0][2],
  ]);

  const axes = [
    quad.normal,    // quad face normal
    obb.axes[0],    // OBB axis 0
    obb.axes[1],    // OBB axis 1
    obb.axes[2],    // OBB axis 2
  ];
  if (e0) axes.push(cross(obb.axes[0], e0), cross(obb.axes[1], e0), cross(obb.axes[2], e0));
  if (e1) axes.push(cross(obb.axes[0], e1), cross(obb.axes[1], e1), cross(obb.axes[2], e1));

  let maxGap = -Infinity;
  for (const axis of axes) {
    const gap = testAxis(obb, verts, axis);
    if (gap > maxGap) maxGap = gap;
  }

  if (maxGap > 0) return { collides: false, clearance: maxGap };
  return { collides: true, clearance: maxGap };   // maxGap is ≤ 0 here
}

/**
 * Tests OBB against every quad in the array.
 * Returns { collides, minClearance, contactQuads }
 *   minClearance: signed min clearance across all quads — negative when penetrating (0 if array is empty)
 */
export function checkCollisions(obb, collisionQuads) {
  let collides = false;
  let minClearance = Infinity;
  const contactQuads = [];

  for (const quad of collisionQuads) {
    const result = testOBBvsQuad(obb, quad);
    if (result.collides) {
      collides = true;
      contactQuads.push(quad);
    }
    if (result.clearance < minClearance) {
      minClearance = result.clearance;
    }
  }

  return {
    collides,
    minClearance: minClearance === Infinity ? 0 : minClearance,
    contactQuads,
  };
}
