# PivotSim Phase 1.5 — Box Model + Collision Detection

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a semi-transparent box to the stairwell scene with real-time SAT collision detection and color-coded clearance feedback.

**Architecture:** A pure-math `box.js` computes OBBs (8 corners + 3 axes) from pose; a pure-math `collision.js` runs SAT against the existing collision quads; `config-panel.js` gains box dims and pose sections; `main.js` wires them together. No new Three.js abstractions needed — the existing quad/scene patterns extend naturally.

**Tech Stack:** Three.js (box mesh only), Vitest/jsdom, vanilla JS ES modules.

---

## File Structure

| File | Action | Purpose |
|------|--------|---------|
| `src/defaults.js` | Modify | Add `BOX_DEFAULTS` and `BOX_POSE_DEFAULTS` exports |
| `src/geometry/box.js` | Create | Box mesh creation, OBB computation, corner extraction |
| `src/geometry/box.test.js` | Create | Tests for OBB math |
| `src/solver/collision.js` | Create | SAT OBB-vs-quad + multi-quad checker |
| `src/solver/collision.test.js` | Create | Tests for SAT correctness including rotated OBB |
| `src/ui/config-panel.js` | Modify | Add box dims section + box pose section; extend return object |
| `src/ui/config-panel.test.js` | Modify | Test new getter/callback methods |
| `src/main.js` | Modify | Create box, wire dims/pose callbacks, update collision on change |
| `index.html` | Modify | Add clearance readout overlay |

---

### Task 1: Box defaults

**Files:**
- Modify: `src/defaults.js`

- [ ] **Step 1: Add box exports to defaults.js**

Append to the end of `src/defaults.js`:

```js
export const BOX_DEFAULTS = {
  length: 2.0,   // m, longest dimension (runs along stairwell Z axis at rest)
  width: 0.8,    // m, horizontal cross-section
  height: 0.5,   // m, vertical dimension
};

export const BOX_POSE_DEFAULTS = {
  x: 0.0,
  y: 0.25,    // half of default height — box sits on floor
  z: -1.0,    // 1m into bottom hallway
  yaw: 0,     // degrees, rotation around Y (vertical) axis
  pitch: 0,   // degrees, rotation around X axis
  roll: 0,    // degrees, rotation around Z axis
};
```

- [ ] **Step 2: Verify tests still pass**

Run: `npx vitest run`
Expected: All existing tests pass (no change to DEFAULTS, just new exports).

- [ ] **Step 3: Commit**

```bash
git add src/defaults.js
git commit -m "feat: add BOX_DEFAULTS and BOX_POSE_DEFAULTS"
```

---

### Task 2: Box model

**Files:**
- Create: `src/geometry/box.js`
- Create: `src/geometry/box.test.js`

#### Coordinate conventions (read before implementing)

`BoxGeometry(width, height, length)` places:
- Width along local X (axis 0)
- Height along local Y (axis 1)
- Length along local Z (axis 2)

So `halfExtents = [width/2, height/2, length/2]` and `getHalfExtents({length, width, height})` returns them in that order.

`computeOBB` takes pose angles in **radians**. `main.js` converts from the panel's degrees. The Euler order `'YXZ'` means: rotate Y by `yaw`, then X by `pitch`, then Z by `roll`. Use `new THREE.Euler(pitch, yaw, roll, 'YXZ')` (Three.js Euler constructor is `(x, y, z, order)`).

Three.js Matrix4 elements are **column-major**. Column 0 = elements[0,1,2], column 1 = elements[4,5,6], column 2 = elements[8,9,10]. These are the world-space directions of the box's local X, Y, Z axes.

- [ ] **Step 1: Write the failing tests**

Create `src/geometry/box.test.js`:

```js
import { describe, it, expect } from 'vitest';
import { computeOBB, getOBBCorners, getHalfExtents } from './box.js';

describe('getHalfExtents', () => {
  it('returns [width/2, height/2, length/2]', () => {
    expect(getHalfExtents({ length: 2.0, width: 0.8, height: 0.5 }))
      .toEqual([0.4, 0.25, 1.0]);
  });
});

describe('computeOBB', () => {
  it('at zero rotation produces identity axes', () => {
    const obb = computeOBB(
      { x: 1, y: 2, z: 3, yaw: 0, pitch: 0, roll: 0 },
      [0.4, 0.25, 1.0]
    );
    expect(obb.center).toEqual([1, 2, 3]);
    expect(obb.halfExtents).toEqual([0.4, 0.25, 1.0]);
    expect(obb.axes[0][0]).toBeCloseTo(1);
    expect(obb.axes[0][1]).toBeCloseTo(0);
    expect(obb.axes[0][2]).toBeCloseTo(0);
    expect(obb.axes[1][0]).toBeCloseTo(0);
    expect(obb.axes[1][1]).toBeCloseTo(1);
    expect(obb.axes[1][2]).toBeCloseTo(0);
    expect(obb.axes[2][0]).toBeCloseTo(0);
    expect(obb.axes[2][1]).toBeCloseTo(0);
    expect(obb.axes[2][2]).toBeCloseTo(1);
  });

  it('90° yaw rotates local X to world -Z and local Z to world +X', () => {
    const obb = computeOBB(
      { x: 0, y: 0, z: 0, yaw: Math.PI / 2, pitch: 0, roll: 0 },
      [0.4, 0.25, 1.0]
    );
    // local X (width axis) → world -Z
    expect(obb.axes[0][0]).toBeCloseTo(0);
    expect(obb.axes[0][1]).toBeCloseTo(0);
    expect(obb.axes[0][2]).toBeCloseTo(-1);
    // local Z (length axis) → world +X
    expect(obb.axes[2][0]).toBeCloseTo(1);
    expect(obb.axes[2][1]).toBeCloseTo(0);
    expect(obb.axes[2][2]).toBeCloseTo(0);
  });
});

describe('getOBBCorners', () => {
  it('returns 8 corners', () => {
    const obb = {
      center: [0, 0, 0],
      axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      halfExtents: [1, 2, 3],
    };
    expect(getOBBCorners(obb).length).toBe(8);
  });

  it('identity OBB corners span ±halfExtents on each axis', () => {
    const obb = {
      center: [0, 0, 0],
      axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      halfExtents: [1, 2, 3],
    };
    const corners = getOBBCorners(obb);
    const xs = corners.map((c) => c[0]);
    const ys = corners.map((c) => c[1]);
    const zs = corners.map((c) => c[2]);
    expect(Math.min(...xs)).toBeCloseTo(-1);
    expect(Math.max(...xs)).toBeCloseTo(1);
    expect(Math.min(...ys)).toBeCloseTo(-2);
    expect(Math.max(...ys)).toBeCloseTo(2);
    expect(Math.min(...zs)).toBeCloseTo(-3);
    expect(Math.max(...zs)).toBeCloseTo(3);
  });

  it('rotated OBB corners are in world space', () => {
    // 90° yaw: local Z (halfExtent=3) now points in world +X
    const obb = {
      center: [0, 0, 0],
      axes: [[0, 0, -1], [0, 1, 0], [1, 0, 0]], // 90° yaw
      halfExtents: [1, 2, 3],
    };
    const corners = getOBBCorners(obb);
    const xs = corners.map((c) => c[0]);
    // local Z (halfExtent=3) is now world +X, local X (halfExtent=1) is world -Z
    // World X span: ±3 (from length axis)
    expect(Math.min(...xs)).toBeCloseTo(-3);
    expect(Math.max(...xs)).toBeCloseTo(3);
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `npx vitest run src/geometry/box.test.js`
Expected: FAIL with module not found

- [ ] **Step 3: Implement box.js**

Create `src/geometry/box.js`:

```js
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `npx vitest run src/geometry/box.test.js`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add src/geometry/box.js src/geometry/box.test.js
git commit -m "feat: box model — OBB computation and corner extraction"
```

---

### Task 3: SAT collision detector

**Files:**
- Create: `src/solver/collision.js`
- Create: `src/solver/collision.test.js`

#### SAT theory (read before implementing)

For OBB vs convex quad, test these axes for separation:
1. Quad face normal (1 axis)
2. OBB's 3 face normals — its `axes[0]`, `axes[1]`, `axes[2]` (3 axes)
3. Cross products of each OBB axis with each of the quad's 2 edge directions (6 axes)

Total: 10 axes. If **any** axis separates the shapes, there is **no collision**. If **all** axes show overlap, there is a collision.

For each axis `L`:
- OBB projection: center = `C·L`, radius = `Σ hᵢ |Aᵢ·L|` → interval `[center-r, center+r]`
- Quad projection: `min/max of {Vᵢ·L}` for the 4 vertices
- Gap = `max(obbMin - quadMax, quadMin - obbMax)`: positive → separated, negative → overlapping

`clearance` returned when not colliding equals `maxGap` across all tested axes — this is the separation on the most-separating axis, which is a lower bound on true geometric clearance (good enough for color coding).

Degenerate cross product axes (length < 1e-10) are skipped by returning `-Infinity` from `testAxis`.

- [ ] **Step 1: Write the failing tests**

Create `src/solver/collision.test.js`:

```js
import { describe, it, expect } from 'vitest';
import { testOBBvsQuad, checkCollisions } from './collision.js';

const SQRT_HALF = Math.SQRT1_2; // ≈ 0.7071

// ─── Shared fixtures ────────────────────────────────────────────────────────

// Floor at y=0, 2m × 2m centered at origin
const floorQuad = {
  type: 'floor',
  vertices: [[-1, 0, -1], [1, 0, -1], [1, 0, 1], [-1, 0, 1]],
  normal: [0, 1, 0],
};

// Ceiling at y=2.4
const ceilQuad = {
  type: 'ceiling',
  vertices: [[-1, 2.4, -1], [1, 2.4, -1], [1, 2.4, 1], [-1, 2.4, 1]],
  normal: [0, -1, 0],
};

// Vertical wall at x=1, spanning y 0–2, z –2 to 2
const wallQuad = {
  type: 'wall-left',
  vertices: [[1, 0, -2], [1, 0, 2], [1, 2, 2], [1, 2, -2]],
  normal: [-1, 0, 0],
};

// Identity OBB well above the floor
const clearOBB = {
  center: [0, 2, 0],
  axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  halfExtents: [0.25, 0.25, 0.25],
};

// Identity OBB centred on the floor plane → collision
const collidingOBB = {
  center: [0, 0, 0],
  axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  halfExtents: [0.25, 0.25, 0.25],
};

// OBB with 45° yaw, long in its local Z (world [+X,0,+X] direction)
const rotatedOBB_far = {
  center: [-1.0, 1.0, 0.0],
  axes: [[SQRT_HALF, 0, -SQRT_HALF], [0, 1, 0], [SQRT_HALF, 0, SQRT_HALF]],
  halfExtents: [0.3, 0.5, 2.0],
};

const rotatedOBB_close = {
  center: [0.0, 1.0, 0.0],
  axes: [[SQRT_HALF, 0, -SQRT_HALF], [0, 1, 0], [SQRT_HALF, 0, SQRT_HALF]],
  halfExtents: [0.3, 0.5, 2.0],
};

// ─── testOBBvsQuad ──────────────────────────────────────────────────────────

describe('testOBBvsQuad', () => {
  it('reports no collision for OBB above floor', () => {
    const result = testOBBvsQuad(clearOBB, floorQuad);
    expect(result.collides).toBe(false);
    expect(result.clearance).toBeCloseTo(1.75, 4);
  });

  it('reports collision for OBB penetrating floor', () => {
    const result = testOBBvsQuad(collidingOBB, floorQuad);
    expect(result.collides).toBe(true);
  });

  it('reports no collision for rotated OBB past the wall', () => {
    // 45°-yaw OBB centred at x=-1, extends ~1.626 in world +X → max reach x≈0.626 < 1
    const result = testOBBvsQuad(rotatedOBB_far, wallQuad);
    expect(result.collides).toBe(false);
    expect(result.clearance).toBeCloseTo(0.374, 2);
  });

  it('reports collision for rotated OBB overlapping the wall', () => {
    // Same OBB centred at x=0 → max reach x≈1.626 > 1
    const result = testOBBvsQuad(rotatedOBB_close, wallQuad);
    expect(result.collides).toBe(true);
  });
});

// ─── checkCollisions ────────────────────────────────────────────────────────

describe('checkCollisions', () => {
  it('returns no collision when OBB clears all quads', () => {
    const result = checkCollisions(clearOBB, [floorQuad, ceilQuad]);
    expect(result.collides).toBe(false);
    expect(result.contactQuads).toHaveLength(0);
  });

  it('minClearance is the closest gap across all quads', () => {
    // clearOBB centre at y=2.0, halfExtent 0.25
    // → gap to floor (y=0) on normal [0,1,0]: 1.75
    // → gap to ceil (y=2.4) on normal [0,-1,0]: 0.15
    const result = checkCollisions(clearOBB, [floorQuad, ceilQuad]);
    expect(result.minClearance).toBeCloseTo(0.15, 2);
  });

  it('reports collision and identifies which quads are hit', () => {
    const farCeil = {
      type: 'ceiling',
      vertices: [[-1, 5, -1], [1, 5, -1], [1, 5, 1], [-1, 5, 1]],
      normal: [0, -1, 0],
    };
    const result = checkCollisions(collidingOBB, [floorQuad, farCeil]);
    expect(result.collides).toBe(true);
    expect(result.contactQuads).toContain(floorQuad);
    expect(result.contactQuads).not.toContain(farCeil);
  });

  it('returns minClearance 0 when colliding', () => {
    const result = checkCollisions(collidingOBB, [floorQuad]);
    expect(result.collides).toBe(true);
    expect(result.minClearance).toBe(0);
  });
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `npx vitest run src/solver/collision.test.js`
Expected: FAIL — module not found

- [ ] **Step 3: Implement collision.js**

Create `src/solver/collision.js`:

```js
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
 *   collides=true  → clearance = 0
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
  return { collides: true, clearance: 0 };
}

/**
 * Tests OBB against every quad in the array.
 * Returns { collides, minClearance, contactQuads }
 *   minClearance: min clearance across non-colliding quads (0 if colliding)
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
    } else if (result.clearance < minClearance) {
      minClearance = result.clearance;
    }
  }

  return {
    collides,
    minClearance: collides ? 0 : (minClearance === Infinity ? 0 : minClearance),
    contactQuads,
  };
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `npx vitest run src/solver/collision.test.js`
Expected: PASS (8 tests)

- [ ] **Step 5: Run full test suite**

Run: `npx vitest run`
Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/solver/collision.js src/solver/collision.test.js
git commit -m "feat: SAT OBB-vs-quad collision detector with clearance measurement"
```

---

### Task 4: Config panel box sections

**Files:**
- Modify: `src/ui/config-panel.js`
- Modify: `src/ui/config-panel.test.js`

The config panel gains two new sections below the existing Display section:
- **Box Dimensions** — three number inputs (length, width, height)
- **Box Pose** — six number inputs (x, y, z, yaw°, pitch°, roll°)

The return object gains `getBoxDims()`, `getBoxPose()`, `onBoxDimsChange(cb)`, `onBoxPoseChange(cb)`.

Both new sections use the same debounce pattern as the stairwell params.

- [ ] **Step 1: Write the failing tests**

Add to the bottom of `src/ui/config-panel.test.js` (inside the existing `describe` block, before the closing `});`):

```js
  it('getBoxDims returns default box dimensions', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    const dims = panel.getBoxDims();
    expect(dims.length).toBe(2.0);
    expect(dims.width).toBe(0.8);
    expect(dims.height).toBe(0.5);
  });

  it('getBoxPose returns default box pose', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    const pose = panel.getBoxPose();
    expect(pose.y).toBeCloseTo(0.25);
    expect(pose.z).toBeCloseTo(-1.0);
    expect(pose.yaw).toBe(0);
  });

  it('onBoxDimsChange and onBoxPoseChange are functions', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    expect(typeof panel.onBoxDimsChange).toBe('function');
    expect(typeof panel.onBoxPoseChange).toBe('function');
  });

  it('onBoxDimsChange is called after a box dim input changes', async () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const cb = vi.fn();
    panel.onBoxDimsChange(cb);

    // Box dim inputs follow all the stairwell inputs; find by label text
    const labels = Array.from(container.querySelectorAll('label'));
    const lengthLabel = labels.find((l) => l.textContent === 'Length (m)');
    const input = lengthLabel.nextElementSibling;
    input.value = '1.5';
    input.dispatchEvent(new Event('input'));

    await new Promise((resolve) => setTimeout(resolve, 150));
    expect(cb).toHaveBeenCalledWith(expect.objectContaining({ length: 1.5 }));
  });
```

- [ ] **Step 2: Run tests to verify new ones fail**

Run: `npx vitest run src/ui/config-panel.test.js`
Expected: 4 new tests FAIL, 5 existing tests still PASS.

- [ ] **Step 3: Implement config-panel.js additions**

At the top of `src/ui/config-panel.js`, add imports:

```js
import { BOX_DEFAULTS, BOX_POSE_DEFAULTS } from '../defaults.js';
```

Inside `createConfigPanel`, add these declarations right after `let debounceTimer = null;`:

```js
  let boxDimsTimer = null;
  let boxPoseTimer = null;
  let boxDimsCallback = null;
  let boxPoseCallback = null;
  const boxDims = { ...BOX_DEFAULTS };
  const boxPose = { ...BOX_POSE_DEFAULTS };
```

At the end of `createConfigPanel`, just before the `return` statement, add the box sections:

```js
  // ── Box Dimensions ──────────────────────────────────────────────────────
  const boxSep = document.createElement('hr');
  boxSep.style.cssText = 'border-color:#334; margin:16px 0;';
  container.appendChild(boxSep);

  const boxDimsTitle = document.createElement('h3');
  boxDimsTitle.textContent = 'Box Dimensions';
  boxDimsTitle.style.cssText = 'font-size:14px; color:#64ffda; margin-bottom:8px;';
  container.appendChild(boxDimsTitle);

  for (const def of [
    { key: 'length', label: 'Length (m)', min: 0.3, max: 4.0, step: 0.05 },
    { key: 'width',  label: 'Width (m)',  min: 0.3, max: 2.0, step: 0.05 },
    { key: 'height', label: 'Height (m)', min: 0.1, max: 1.5, step: 0.05 },
  ]) {
    const wrapper = document.createElement('div');
    wrapper.style.cssText = 'margin-bottom:12px;';
    const lbl = document.createElement('label');
    lbl.textContent = def.label;
    lbl.style.cssText = 'display:block; font-size:12px; margin-bottom:4px; color:#aaa;';
    wrapper.appendChild(lbl);
    const inp = document.createElement('input');
    inp.type = 'number';
    inp.value = boxDims[def.key];
    inp.min = def.min;
    inp.max = def.max;
    inp.step = def.step;
    inp.style.cssText = 'width:100%; padding:4px 8px; background:#0d1b2a; color:#e0e0e0; border:1px solid #334; border-radius:4px;';
    inp.addEventListener('input', () => {
      boxDims[def.key] = Number(inp.value);
      clearTimeout(boxDimsTimer);
      boxDimsTimer = setTimeout(() => { if (boxDimsCallback) boxDimsCallback({ ...boxDims }); }, 100);
    });
    wrapper.appendChild(inp);
    container.appendChild(wrapper);
  }

  // ── Box Pose ────────────────────────────────────────────────────────────
  const poseSep = document.createElement('hr');
  poseSep.style.cssText = 'border-color:#334; margin:16px 0;';
  container.appendChild(poseSep);

  const boxPoseTitle = document.createElement('h3');
  boxPoseTitle.textContent = 'Box Pose';
  boxPoseTitle.style.cssText = 'font-size:14px; color:#64ffda; margin-bottom:8px;';
  container.appendChild(boxPoseTitle);

  for (const def of [
    { key: 'x',     label: 'X (m)',      min: -6, max: 6,    step: 0.05 },
    { key: 'y',     label: 'Y (m)',      min: -1, max: 5,    step: 0.05 },
    { key: 'z',     label: 'Z (m)',      min: -6, max: 6,    step: 0.05 },
    { key: 'yaw',   label: 'Yaw (°)',    min: -180, max: 180, step: 1 },
    { key: 'pitch', label: 'Pitch (°)',  min: -90,  max: 90,  step: 1 },
    { key: 'roll',  label: 'Roll (°)',   min: -180, max: 180, step: 1 },
  ]) {
    const wrapper = document.createElement('div');
    wrapper.style.cssText = 'margin-bottom:12px;';
    const lbl = document.createElement('label');
    lbl.textContent = def.label;
    lbl.style.cssText = 'display:block; font-size:12px; margin-bottom:4px; color:#aaa;';
    wrapper.appendChild(lbl);
    const inp = document.createElement('input');
    inp.type = 'number';
    inp.value = boxPose[def.key];
    inp.min = def.min;
    inp.max = def.max;
    inp.step = def.step;
    inp.style.cssText = 'width:100%; padding:4px 8px; background:#0d1b2a; color:#e0e0e0; border:1px solid #334; border-radius:4px;';
    inp.addEventListener('input', () => {
      boxPose[def.key] = Number(inp.value);
      clearTimeout(boxPoseTimer);
      boxPoseTimer = setTimeout(() => { if (boxPoseCallback) boxPoseCallback({ ...boxPose }); }, 100);
    });
    wrapper.appendChild(inp);
    container.appendChild(wrapper);
  }
```

Update the `return` statement to add the four new methods:

```js
  return {
    getParams() {
      return { ...params };
    },
    getBoxDims() {
      return { ...boxDims };
    },
    getBoxPose() {
      return { ...boxPose };
    },
    onCeilingToggle(callback) {
      ceilCheck.addEventListener('change', () => callback(ceilCheck.checked));
    },
    onQuadDebugToggle(callback) {
      quadDebugCheck.addEventListener('change', () => callback(quadDebugCheck.checked));
    },
    onBoxDimsChange(callback) {
      boxDimsCallback = callback;
    },
    onBoxPoseChange(callback) {
      boxPoseCallback = callback;
    },
  };
```

**Note:** The existing return has `onCeilingToggle` and `onQuadDebugToggle` — replace the entire return block with the version above.

- [ ] **Step 4: Run tests to verify they pass**

Run: `npx vitest run src/ui/config-panel.test.js`
Expected: PASS (all 9 tests)

- [ ] **Step 5: Run full suite**

Run: `npx vitest run`
Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/ui/config-panel.js src/ui/config-panel.test.js src/defaults.js
git commit -m "feat: add box dims and pose sections to config panel"
```

---

### Task 5: Wire box into scene

**Files:**
- Modify: `src/main.js`
- Modify: `index.html`

#### What changes

1. `index.html`: add `position:relative` to `#viewport`, add `#clearance-readout` overlay div inside it.
2. `main.js`:
   - Import box and collision modules + new defaults
   - Track `currentCollisionQuads` (set in `rebuildStairwell`)
   - Create box mesh on init, re-create when dims change
   - Update box mesh pose and collision on pose change
   - `updateBoxCollision()` — compute OBB, run SAT, update box color + readout text

There are no new tests for this task (it's glue code that requires the browser; the unit-testable math is in box.js and collision.js).

- [ ] **Step 1: Update index.html**

Replace the `#viewport` div (line 43):

```html
    <div id="viewport" style="position:relative;">
      <div id="clearance-readout"
           style="position:absolute; top:16px; left:16px; z-index:10;
                  background:rgba(0,0,0,0.7); color:#22ff88; font-size:13px;
                  font-family:monospace; padding:6px 14px; border-radius:4px;
                  pointer-events:none; display:none;">
        Clear
      </div>
    </div>
```

- [ ] **Step 2: Rewrite main.js**

Replace `src/main.js` entirely:

```js
import * as THREE from 'three';
import { createScene } from './viewer/scene.js';
import { createConfigPanel } from './ui/config-panel.js';
import { buildStairwell } from './geometry/stairwell.js';
import { buildQuadDebug } from './viewer/debug.js';
import { createBoxMesh, updateBoxMeshPose, computeOBB, getHalfExtents } from './geometry/box.js';
import { checkCollisions } from './solver/collision.js';
import { DEFAULTS, BOX_DEFAULTS, BOX_POSE_DEFAULTS } from './defaults.js';

const DEG = Math.PI / 180;

const viewport = document.getElementById('viewport');
const configContainer = document.getElementById('config-panel');
const readout = document.getElementById('clearance-readout');

const { scene, camera, renderer, controls } = createScene(viewport);

let currentStairwell = null;
let currentQuadDebug = null;
let currentBox = null;
let currentCollisionQuads = [];
let currentBoxDims = { ...BOX_DEFAULTS };
let currentBoxPose = { ...BOX_POSE_DEFAULTS };

// Convert pose from degrees (UI) to radians (math)
function poseRad(pose) {
  return {
    x: pose.x, y: pose.y, z: pose.z,
    yaw: pose.yaw * DEG,
    pitch: pose.pitch * DEG,
    roll: pose.roll * DEG,
  };
}

function updateBoxCollision() {
  if (!currentBox || currentCollisionQuads.length === 0) return;
  const obb = computeOBB(poseRad(currentBoxPose), getHalfExtents(currentBoxDims));
  const { collides, minClearance } = checkCollisions(obb, currentCollisionQuads);

  if (collides) {
    currentBox.material.color.setHex(0xff2222);
    readout.textContent = 'COLLISION';
    readout.style.color = '#ff4444';
  } else if (minClearance < 0.05) {
    currentBox.material.color.setHex(0xffaa00);
    readout.textContent = `Tight: ${(minClearance * 100).toFixed(1)} cm`;
    readout.style.color = '#ffaa00';
  } else {
    currentBox.material.color.setHex(0x22ff88);
    readout.textContent = `Clear: ${(minClearance * 100).toFixed(0)} cm`;
    readout.style.color = '#22ff88';
  }
}

function rebuildStairwell(params) {
  if (currentStairwell) {
    scene.remove(currentStairwell);
    currentStairwell.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const prevQuadDebugVisible = currentQuadDebug ? currentQuadDebug.visible : false;
  if (currentQuadDebug) {
    scene.remove(currentQuadDebug);
    currentQuadDebug.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const { group, collisionQuads } = buildStairwell(params);
  currentCollisionQuads = collisionQuads;
  scene.add(group);
  currentStairwell = group;

  currentQuadDebug = buildQuadDebug(collisionQuads);
  currentQuadDebug.visible = prevQuadDebugVisible;
  scene.add(currentQuadDebug);

  // Auto-frame camera
  const box = new THREE.Box3().setFromObject(group);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z);
  controls.target.copy(center);
  camera.position.set(
    center.x + maxDim * 1.2,
    center.y + maxDim * 0.8,
    center.z + maxDim * 1.2
  );
  controls.update();

  updateBoxCollision();
}

function rebuildBox() {
  if (currentBox) {
    scene.remove(currentBox);
    currentBox.geometry.dispose();
    currentBox.material.dispose();
  }
  currentBox = createBoxMesh(currentBoxDims);
  updateBoxMeshPose(currentBox, poseRad(currentBoxPose));
  scene.add(currentBox);
  updateBoxCollision();
}

const panel = createConfigPanel(configContainer, { ...DEFAULTS }, (params) => {
  rebuildStairwell(params);
});

panel.onCeilingToggle((visible) => {
  if (!currentStairwell) return;
  currentStairwell.traverse((child) => {
    if (child.userData.isSurface) child.visible = visible;
  });
});

panel.onQuadDebugToggle((visible) => {
  if (currentQuadDebug) currentQuadDebug.visible = visible;
});

panel.onBoxDimsChange((dims) => {
  currentBoxDims = dims;
  rebuildBox();
});

panel.onBoxPoseChange((pose) => {
  currentBoxPose = pose;
  if (currentBox) {
    updateBoxMeshPose(currentBox, poseRad(pose));
    updateBoxCollision();
  }
});

// Initial build
rebuildStairwell(panel.getParams());
rebuildBox();
readout.style.display = 'block';

// Animation loop
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// Resize handler
window.addEventListener('resize', () => {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});
```

- [ ] **Step 3: Run all tests**

Run: `npx vitest run`
Expected: All tests pass (main.js has no tests — only glue code).

- [ ] **Step 4: Visual verification**

Run: `npm run dev`

Verify:
1. Box appears in the bottom hallway — semi-transparent green
2. Clearance readout shows "Clear: XX cm" in top-left of viewport
3. Moving Z input toward 0 (into stairs) → box enters stairwell, clearance decreases
4. Pushing box into a wall/step → readout turns red and shows "COLLISION", box turns red
5. Box near a wall (< 5cm clearance) → box and readout turn orange
6. Changing box Length/Width/Height → box mesh updates, collision re-checks
7. Yaw input at 45° → box rotates, collision updates correctly
8. Changing stairwell params → geometry rebuilds, box collision re-checks with new quads

- [ ] **Step 5: Commit**

```bash
git add src/main.js index.html
git commit -m "feat: wire box into scene with real-time SAT collision feedback"
```

---

## Verification Checklist

After all tasks:

1. `npx vitest run` — all tests pass (target: ~40 tests)
2. `npm run dev` — box appears in bottom hallway, green
3. Box dims/pose inputs in config panel update box in real-time
4. Collision colors: green → orange (< 5cm) → red (collision)
5. Clearance readout updates in real-time
6. Changing stairwell params triggers collision re-check
7. Yaw rotation works correctly (try 45° and 90°)
