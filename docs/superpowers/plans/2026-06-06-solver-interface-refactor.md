# Solver Interface Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decouple the solver from the SA implementation by introducing a `Planner` interface, extracting shared pose math to `utils.js`, moving context-building to `context.js`, and reducing `trajectory.js` to a thin timing layer.

**Architecture:** A single generic `worker.js` holds a planner registry (`{ sa: saPlanner }`) and handles all message protocol. Each planner module exports `{ plan(context, config, onProgress, shouldCancel), formatProgress(data, container) }`. The worker builds a `PlannerContext` via `buildPlannerContext()`, calls `planner.plan()`, then wraps the raw path in `buildTrajectory()` to add timing before posting the result.

**Tech Stack:** Vanilla JS ES modules, Vite, Vitest, Three.js (geometry only — no GL in tests), jsdom test environment.

---

## File Map

| Status | File | Change |
|--------|------|--------|
| Create | `src/solver/utils.js` | Shared pose math extracted from `trajectory.js` |
| Create | `src/solver/utils.test.js` | Tests for all utils exports |
| Create | `src/solver/context.js` | `buildPlannerContext()` |
| Create | `src/solver/context.test.js` | Tests for context shape |
| Create | `src/solver/planners/sa.js` | SA planner: `{ plan, formatProgress, evalSegment, DEFAULT_WEIGHTS }` |
| Create | `src/solver/planners/sa.test.js` | `plan()` + `evalSegment` tests |
| Modify | `src/solver/trajectory.js` | Replace with `buildTrajectory(poses)` only |
| Modify | `src/solver/trajectory.test.js` | Replace with `buildTrajectory` tests only |
| Modify | `src/solver/worker.js` | Generic worker with planner registry |
| Modify | `src/ui/timeline.js` | `renderSolving()` placeholder + `updateProgress(data, formatter)` |
| Modify | `src/main.js` | Update `lerpPose` import; add `saPlanner` import; update progress handler |

---

## Task 1: Extract shared pose math to `utils.js`

**Files:**
- Create: `src/solver/utils.js`
- Create: `src/solver/utils.test.js`
- Modify: `src/solver/trajectory.js` (lines 1–52, 59–63)
- Modify: `src/main.js` (line 10)

- [ ] **Step 1: Write the failing tests**

Create `src/solver/utils.test.js`:

```js
import { describe, it, expect } from 'vitest';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose,
  applyRotationPropagation, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
} from './utils.js';

describe('euclideanDelta', () => {
  it('computes 3D distance', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 3, y: 4, z: 0, yaw: 0, pitch: 0, roll: 0 };
    expect(euclideanDelta(a, b)).toBeCloseTo(5, 5);
  });
});

describe('angularDelta', () => {
  it('sums absolute differences in yaw/pitch/roll', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0,   pitch: 0,   roll: 0   };
    const b = { x: 0, y: 0, z: 0, yaw: 0.5, pitch: 0.3, roll: 0.1 };
    expect(angularDelta(a, b)).toBeCloseTo(0.9, 5);
  });
});

describe('segmentDuration', () => {
  it('uses linear speed for position-dominant segments', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 1, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    expect(segmentDuration(a, b)).toBeCloseTo(1 / MAX_LINEAR_SPEED, 5);
  });

  it('uses angular speed for rotation-dominant segments', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0,                pitch: 0, roll: 0 };
    const b = { x: 0, y: 0, z: 0, yaw: MAX_ANGULAR_SPEED, pitch: 0, roll: 0 };
    expect(segmentDuration(a, b)).toBeCloseTo(1.0, 5);
  });
});

describe('lerpPose', () => {
  const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
  const b = { x: 2, y: 4, z: 6, yaw: 1, pitch: 0.5, roll: -0.5 };

  it('returns a at t=0', () => {
    const r = lerpPose(a, b, 0);
    expect(r.x).toBeCloseTo(0); expect(r.yaw).toBeCloseTo(0);
  });

  it('returns b at t=1', () => {
    const r = lerpPose(a, b, 1);
    expect(r.x).toBeCloseTo(2); expect(r.yaw).toBeCloseTo(1);
  });

  it('returns midpoint at t=0.5', () => {
    const r = lerpPose(a, b, 0.5);
    expect(r.x).toBeCloseTo(1); expect(r.z).toBeCloseTo(3); expect(r.yaw).toBeCloseTo(0.5);
  });
});

describe('applyRotationPropagation', () => {
  function makePoses(n) {
    return Array.from({ length: n }, (_, i) => ({
      x: i, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0,
    }));
  }

  it('shifts yaw for indices in [startIdx, endIdx)', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 0.5);
    expect(result[0].yaw).toBeCloseTo(0);
    expect(result[1].yaw).toBeCloseTo(0);
    expect(result[2].yaw).toBeCloseTo(0.5);
    expect(result[3].yaw).toBeCloseTo(0.5);
    expect(result[4].yaw).toBeCloseTo(0);
  });

  it('leaves x/y/z untouched', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 1, 4, 'pitch', 1.0);
    result.forEach((p, i) => {
      expect(p.x).toBeCloseTo(i);
      expect(p.y).toBeCloseTo(0);
      expect(p.z).toBeCloseTo(0);
    });
  });

  it('does not mutate the original poses array', () => {
    const poses = makePoses(3);
    applyRotationPropagation(poses, 1, 2, 'roll', 0.3);
    expect(poses[1].roll).toBeCloseTo(0);
  });

  it('forward range: startIdx=i, endIdx=n-1 shifts all interior keypoints from i onward', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 1.0);
    expect(result[1].yaw).toBeCloseTo(0);
    expect(result[2].yaw).toBeCloseTo(1);
    expect(result[3].yaw).toBeCloseTo(1);
    expect(result[4].yaw).toBeCloseTo(0);
  });

  it('backward range: startIdx=1, endIdx=i+1 shifts all interior keypoints up to i', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 1, 3, 'pitch', 0.7);
    expect(result[0].pitch).toBeCloseTo(0);
    expect(result[1].pitch).toBeCloseTo(0.7);
    expect(result[2].pitch).toBeCloseTo(0.7);
    expect(result[3].pitch).toBeCloseTo(0);
  });
});
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npx vitest run src/solver/utils.test.js
```

Expected: FAIL — `Cannot find module './utils.js'`

- [ ] **Step 3: Create `src/solver/utils.js`**

```js
export const MAX_LINEAR_SPEED  = 0.5;
export const MAX_ANGULAR_SPEED = 0.5;

function angleDiff(from, to) {
  let d = (to - from) % (2 * Math.PI);
  if (d >  Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

export function euclideanDelta(a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  return Math.sqrt(dx*dx + dy*dy + dz*dz);
}

export function angularDelta(a, b) {
  return Math.abs(angleDiff(a.yaw,   b.yaw))
       + Math.abs(angleDiff(a.pitch, b.pitch))
       + Math.abs(angleDiff(a.roll,  b.roll));
}

export function segmentDuration(a, b) {
  return Math.max(
    euclideanDelta(a, b) / MAX_LINEAR_SPEED,
    angularDelta(a, b)   / MAX_ANGULAR_SPEED,
  );
}

export function lerpPose(a, b, t) {
  return {
    x:     a.x     + (b.x     - a.x)     * t,
    y:     a.y     + (b.y     - a.y)     * t,
    z:     a.z     + (b.z     - a.z)     * t,
    yaw:   a.yaw   + angleDiff(a.yaw,   b.yaw)   * t,
    pitch: a.pitch + angleDiff(a.pitch, b.pitch) * t,
    roll:  a.roll  + angleDiff(a.roll,  b.roll)  * t,
  };
}

export function applyRotationPropagation(poses, startIdx, endIdx, dof, delta) {
  return poses.map((p, j) =>
    j >= startIdx && j < endIdx ? { ...p, [dof]: p[dof] + delta } : { ...p }
  );
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
npx vitest run src/solver/utils.test.js
```

Expected: PASS — all 11 tests green.

- [ ] **Step 5: Update `trajectory.js` — swap local definitions for utils imports**

Replace the top of `src/solver/trajectory.js` (lines 1–13 and the function bodies for `euclideanDelta`, `angleDiff`, `angularDelta`, `segmentDuration`, `lerpPose`, `applyRotationPropagation`) with a single import. The file should now start:

```js
import { checkCollisions } from './collision.js';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose, applyRotationPropagation,
  MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
} from './utils.js';

export const DEFAULT_WEIGHTS = {
  w_col: 100, w_clr: 1.5, w_rot: 0.45, w_pos: 0.45, w_time: 0.5, w_void: 150, w_nk: 2,
};

const CLEARANCE_CAP = 0.3;
const DOFS = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'];
const ROTATION_DOFS = ['yaw', 'pitch', 'roll'];
const SIGMA = { x: 0.1, y: 0.1, z: 0.1, yaw: 0.3, pitch: 0.2, roll: 0.2 };
```

Delete the local definitions of `euclideanDelta`, `angleDiff`, `angularDelta`, `segmentDuration`, `lerpPose`, `applyRotationPropagation` (lines 17–63 in the original file). Remove the `getEndpoints` import from `path.js` (it was unused dead code in `optimizeTrajectory`). Everything from `computeOBB` onward stays unchanged.

- [ ] **Step 6: Update `trajectory.test.js` — remove migrated tests**

Replace the entire contents of `src/solver/trajectory.test.js` with only the tests that remain relevant to what `trajectory.js` still exports. Remove tests for `euclideanDelta`, `angularDelta`, `segmentDuration`, `lerpPose`, `applyRotationPropagation` (they now live in `utils.test.js`). Update the import line. The file should now be:

```js
import { describe, it, expect } from 'vitest';
import {
  evalSegment, optimizeTrajectory,
} from './trajectory.js';

const openQuads = [
  { type: 'floor',   vertices: [[-20,0,-20],[20,0,-20],[20,0,20],[-20,0,20]],     normal: [0,1,0]  },
  { type: 'ceiling', vertices: [[-20,10,-20],[20,10,-20],[20,10,20],[-20,10,20]], normal: [0,-1,0] },
];
const tinyHalf = [0.05, 0.05, 0.05];

describe('evalSegment', () => {
  it('returns zero collEnergy for a clear segment far from surfaces', () => {
    const a = { x: 0, y: 5, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 5, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, openQuads, tinyHalf);
    expect(seg.collEnergy).toBe(0);
    expect(seg.clrEnergy).toBeGreaterThan(0);
    expect(seg.duration).toBeGreaterThan(0);
  });

  it('returns positive collEnergy when segment clips a wall', () => {
    const wallQuad = {
      type: 'wall',
      vertices: [[0.1,-5,-5],[0.1,5,-5],[0.1,5,5],[0.1,-5,5]],
      normal: [-1, 0, 0],
    };
    const a = { x: 0, y: 1, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 1, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, [wallQuad], [0.5, 0.25, 0.5]);
    expect(seg.collEnergy).toBeGreaterThan(0);
  });
});

const openCenterline = {
  points: [[0, 0, -5], [0, 0, 5]],
  totalLength: 10,
  ceilingHeight: 2.4,
};

describe('optimizeTrajectory', () => {
  it('returns a TrajectoryResult with correct shape', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 100 }, null, null
    );
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(Array.isArray(result.segmentTimes)).toBe(true);
    expect(result.segmentTimes.length).toBe(result.poses.length - 1);
    expect(typeof result.fits).toBe('boolean');
    expect(typeof result.tightestIndex).toBe('number');
    expect(result.tightestIndex).toBeGreaterThanOrEqual(0);
    expect(result.tightestIndex).toBeLessThan(result.poses.length);
  });

  it('totalTime equals sum of segmentTimes', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 50 }, null, null
    );
    const sum = result.segmentTimes.reduce((a, b) => a + b, 0);
    expect(result.totalTime).toBeCloseTo(sum, 5);
  });

  it('fits === true immediately for an open space with a tiny box', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 10 }, null, null
    );
    expect(result.fits).toBe(true);
  });

  it('calls onProgress with the right shape', async () => {
    const calls = [];
    await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 600 },
      (p) => calls.push(p), null
    );
    expect(calls.length).toBeGreaterThan(0);
    const p = calls[0];
    expect(Array.isArray(p.poses)).toBe(true);
    expect(typeof p.energy).toBe('number');
    expect(typeof p.temperature).toBe('number');
    expect(typeof p.iteration).toBe('number');
  });

  it('respects shouldCancel', async () => {
    let callCount = 0;
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 50000 },
      () => { callCount++; },
      () => callCount >= 1
    );
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(callCount).toBeLessThan(10);
  });
});
```

- [ ] **Step 7: Update `main.js` — change lerpPose import**

In `src/main.js`, change line 10 from:
```js
import { lerpPose } from './solver/trajectory.js';
```
to:
```js
import { lerpPose } from './solver/utils.js';
```

- [ ] **Step 8: Run all tests to verify nothing broke**

```bash
npm test
```

Expected: All tests pass. `utils.test.js` and `trajectory.test.js` both green.

- [ ] **Step 9: Commit**

```bash
git add src/solver/utils.js src/solver/utils.test.js src/solver/trajectory.js src/solver/trajectory.test.js src/main.js
git commit -m "refactor: extract shared pose math to utils.js"
```

---

## Task 2: Create `context.js`

**Files:**
- Create: `src/solver/context.js`
- Create: `src/solver/context.test.js`

- [ ] **Step 1: Write the failing tests**

Create `src/solver/context.test.js`:

```js
import { describe, it, expect } from 'vitest';
import { buildPlannerContext } from './context.js';
import { DEFAULTS, BOX_DEFAULTS } from '../defaults.js';

describe('buildPlannerContext', () => {
  it('returns all required fields', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(Array.isArray(ctx.collisionQuads)).toBe(true);
    expect(ctx.collisionQuads.length).toBeGreaterThan(0);
    expect(Array.isArray(ctx.halfExtents)).toBe(true);
    expect(ctx.halfExtents).toHaveLength(3);
    expect(Array.isArray(ctx.containmentOBBs)).toBe(true);
    expect(ctx.centerline).toBeDefined();
    expect(ctx.startPose).toBeDefined();
    expect(ctx.endPose).toBeDefined();
  });

  it('startPose and endPose are valid 6-DOF poses', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    for (const pose of [ctx.startPose, ctx.endPose]) {
      for (const k of ['x', 'y', 'z', 'yaw', 'pitch', 'roll']) {
        expect(typeof pose[k]).toBe('number');
        expect(Number.isFinite(pose[k])).toBe(true);
      }
    }
  });

  it('endPose is above startPose (stairs go up)', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(ctx.endPose.y).toBeGreaterThan(ctx.startPose.y);
  });

  it('startPose and endPose are distinct', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    const dist = Math.sqrt(
      (ctx.endPose.x - ctx.startPose.x) ** 2 +
      (ctx.endPose.y - ctx.startPose.y) ** 2 +
      (ctx.endPose.z - ctx.startPose.z) ** 2
    );
    expect(dist).toBeGreaterThan(0.5);
  });

  it('halfExtents has positive values matching box dims', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    for (const h of ctx.halfExtents) {
      expect(h).toBeGreaterThan(0);
    }
    expect(ctx.halfExtents[0]).toBeCloseTo(BOX_DEFAULTS.width  / 2, 5);
    expect(ctx.halfExtents[1]).toBeCloseTo(BOX_DEFAULTS.height / 2, 5);
    expect(ctx.halfExtents[2]).toBeCloseTo(BOX_DEFAULTS.length / 2, 5);
  });
});
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npx vitest run src/solver/context.test.js
```

Expected: FAIL — `Cannot find module './context.js'`

- [ ] **Step 3: Create `src/solver/context.js`**

```js
import { buildStairwell } from '../geometry/stairwell.js';
import { getHalfExtents } from '../geometry/box.js';
import { buildCenterline, buildContainmentOBBs } from './path.js';

export function buildPlannerContext(stairwellParams, boxDims) {
  const { collisionQuads } = buildStairwell(stairwellParams);
  const halfExtents        = getHalfExtents(boxDims);
  const centerline         = buildCenterline(stairwellParams);
  const containmentOBBs    = buildContainmentOBBs(centerline, stairwellParams);

  const { points, ceilingHeight } = centerline;
  const halfCeil = ceilingHeight / 2;
  function midpoint(a, b) {
    return [(a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2];
  }
  const startPt = midpoint(points[0], points[1]);
  const endPt   = midpoint(points[points.length - 1], points[points.length - 2]);

  const startPose = { x: startPt[0], y: startPt[1] + halfCeil, z: startPt[2], yaw: 0, pitch: 0, roll: 0 };
  const endPose   = { x: endPt[0],   y: endPt[1]   + halfCeil, z: endPt[2],   yaw: 0, pitch: 0, roll: 0 };

  return { collisionQuads, halfExtents, startPose, endPose, containmentOBBs, centerline };
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
npx vitest run src/solver/context.test.js
```

Expected: PASS — all 5 tests green.

- [ ] **Step 5: Run all tests**

```bash
npm test
```

Expected: All tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/solver/context.js src/solver/context.test.js
git commit -m "feat: add buildPlannerContext to solver/context.js"
```

---

## Task 3: Create `planners/sa.js`

**Files:**
- Create: `src/solver/planners/sa.js`
- Create: `src/solver/planners/sa.test.js`

The SA planner adapts `optimizeTrajectory` to the `{ plan, formatProgress }` interface. The core algorithm is unchanged; only the function signature and progress payload change.

- [ ] **Step 1: Write the failing tests**

Create `src/solver/planners/sa.test.js`:

```js
import { describe, it, expect } from 'vitest';
import { saPlanner, evalSegment, DEFAULT_WEIGHTS } from './sa.js';

// ── Shared fixtures ────────────────────────────────────────────────────────

const openQuads = [
  { type: 'floor',   vertices: [[-20,0,-20],[20,0,-20],[20,0,20],[-20,0,20]],     normal: [0,1,0]  },
  { type: 'ceiling', vertices: [[-20,10,-20],[20,10,-20],[20,10,20],[-20,10,20]], normal: [0,-1,0] },
];
const tinyHalf = [0.05, 0.05, 0.05];

const openContext = {
  collisionQuads: openQuads,
  halfExtents: tinyHalf,
  startPose: { x: 0, y: 1.2, z: -2.5, yaw: 0, pitch: 0, roll: 0 },
  endPose:   { x: 0, y: 1.2, z:  2.5, yaw: 0, pitch: 0, roll: 0 },
  containmentOBBs: [],
  centerline: {
    points: [[0, 0, -5], [0, 0, -1.5], [0, 0, 0], [0, 0, 1.5], [0, 0, 5]],
    totalLength: 10,
    ceilingHeight: 2.4,
  },
};

// ── evalSegment ────────────────────────────────────────────────────────────

describe('evalSegment', () => {
  it('returns zero collEnergy for a clear segment far from surfaces', () => {
    const a = { x: 0, y: 5, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 5, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, openQuads, tinyHalf);
    expect(seg.collEnergy).toBe(0);
    expect(seg.clrEnergy).toBeGreaterThan(0);
    expect(seg.duration).toBeGreaterThan(0);
  });

  it('returns positive collEnergy when segment clips a wall', () => {
    const wallQuad = {
      type: 'wall',
      vertices: [[0.1,-5,-5],[0.1,5,-5],[0.1,5,5],[0.1,-5,5]],
      normal: [-1, 0, 0],
    };
    const a = { x: 0, y: 1, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 1, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, [wallQuad], [0.5, 0.25, 0.5]);
    expect(seg.collEnergy).toBeGreaterThan(0);
  });
});

// ── saPlanner.plan ─────────────────────────────────────────────────────────

describe('saPlanner.plan', () => {
  it('returns PlanResult with correct shape', async () => {
    const result = await saPlanner.plan(openContext, { maxIter: 100 }, null, null);
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(typeof result.fits).toBe('boolean');
    expect(typeof result.tightestIndex).toBe('number');
    expect(result.tightestIndex).toBeGreaterThanOrEqual(0);
    expect(result.tightestIndex).toBeLessThan(result.poses.length);
  });

  it('does not include segmentTimes or totalTime in result', async () => {
    const result = await saPlanner.plan(openContext, { maxIter: 50 }, null, null);
    expect(result.segmentTimes).toBeUndefined();
    expect(result.totalTime).toBeUndefined();
  });

  it('fits === true for open space with tiny box', async () => {
    const result = await saPlanner.plan(openContext, { maxIter: 10 }, null, null);
    expect(result.fits).toBe(true);
  });

  it('calls onProgress with correct shape', async () => {
    const calls = [];
    await saPlanner.plan(openContext, { maxIter: 600 }, (p) => calls.push(p), null);
    expect(calls.length).toBeGreaterThan(0);
    const p = calls[0];
    expect(Array.isArray(p.poses)).toBe(true);
    expect(typeof p.energy).toBe('number');
    expect(typeof p.temperature).toBe('number');
    expect(typeof p.iteration).toBe('number');
    expect(typeof p.maxIter).toBe('number');
  });

  it('respects shouldCancel', async () => {
    let callCount = 0;
    const result = await saPlanner.plan(
      openContext, { maxIter: 50000 },
      () => { callCount++; },
      () => callCount >= 1
    );
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(callCount).toBeLessThan(10);
  });
});
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npx vitest run src/solver/planners/sa.test.js
```

Expected: FAIL — `Cannot find module './sa.js'`

- [ ] **Step 3: Create `src/solver/planners/sa.js`**

```js
import { checkCollisions } from '../collision.js';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose, applyRotationPropagation,
} from '../utils.js';

export const DEFAULT_WEIGHTS = {
  w_col: 100, w_clr: 1.5, w_rot: 0.45, w_pos: 0.45, w_time: 0.5, w_void: 150, w_nk: 2,
};

const CLEARANCE_CAP = 0.3;
const DOFS          = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'];
const ROTATION_DOFS = ['yaw', 'pitch', 'roll'];
const SIGMA         = { x: 0.1, y: 0.1, z: 0.1, yaw: 0.3, pitch: 0.2, roll: 0.2 };

function computeOBB({ x, y, z, yaw, pitch, roll }, halfExtents) {
  const cy = Math.cos(yaw),   sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll),  sr = Math.sin(roll);
  return {
    center: [x, y, z],
    axes: [
      [ cy*cr + sy*sp*sr,  cp*sr, -sy*cr + cy*sp*sr ],
      [-cy*sr + sy*sp*cr,  cp*cr,  sy*sr + cy*sp*cr ],
      [ sy*cp,            -sp,     cy*cp             ],
    ],
    halfExtents,
  };
}

function randn() {
  const u = 1 - Math.random(), v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function signedDistFromOBBs(px, py, pz, obbs) {
  let best = Infinity;
  for (const obb of obbs) {
    const dx = px - obb.center[0], dy = py - obb.center[1], dz = pz - obb.center[2];
    let maxExcess = -Infinity;
    for (let i = 0; i < 3; i++) {
      const ax     = obb.axes[i];
      const proj   = Math.abs(dx * ax[0] + dy * ax[1] + dz * ax[2]);
      const excess = proj - obb.halfExtents[i];
      if (excess > maxExcess) maxExcess = excess;
    }
    if (maxExcess < best) best = maxExcess;
  }
  return best;
}

export function evalSegment(a, b, collisionQuads, halfExtents, containmentOBBs) {
  const dur  = segmentDuration(a, b);
  const dist = euclideanDelta(a, b);
  const K    = Math.max(5, Math.ceil(dur / 0.2), Math.ceil(dist / 0.05));

  let worstC = Infinity, worstT = 0.5;
  const clearances = [];
  let voidEnergy = 0;
  for (let k = 0; k < K; k++) {
    const t    = (k + 0.5) / K;
    const pose = lerpPose(a, b, t);
    const { minClearance } = checkCollisions(computeOBB(pose, halfExtents), collisionQuads);
    clearances.push(minClearance);
    if (minClearance < worstC) { worstC = minClearance; worstT = t; }

    if (containmentOBBs?.length) {
      const excess = signedDistFromOBBs(pose.x, pose.y, pose.z, containmentOBBs);
      if (excess > 0) voidEnergy += excess * excess;
    }
  }

  const halfSpacing = 0.5 / K;
  for (let r = 1; r <= 5; r++) {
    for (const s of [-1, 1]) {
      const t = Math.max(0.01, Math.min(0.99, worstT + s * (r / 5) * halfSpacing));
      const { minClearance } = checkCollisions(
        computeOBB(lerpPose(a, b, t), halfExtents), collisionQuads,
      );
      clearances.push(minClearance);
    }
  }

  let collEnergy = 0, clrEnergy = 0;
  for (const c of clearances) {
    if (c < 0) collEnergy += c * c;
    else        clrEnergy += Math.min(c, CLEARANCE_CAP);
  }
  return { collEnergy, clrEnergy, voidEnergy, duration: dur };
}

function totalEnergy(segData, poses, w) {
  let E = 0;
  for (let i = 0; i < segData.length; i++) {
    E += w.w_col  * segData[i].collEnergy;
    E -= w.w_clr  * segData[i].clrEnergy;
    E += w.w_void * (segData[i].voidEnergy ?? 0);
    E += w.w_rot  * angularDelta(poses[i], poses[i + 1]);
    E += w.w_pos  * euclideanDelta(poses[i], poses[i + 1]);
    E += w.w_time * segData[i].duration;
  }
  E += w.w_nk * (poses.length - 2);
  return E;
}

export const saPlanner = {
  async plan(context, config, onProgress, shouldCancel) {
    const { collisionQuads, halfExtents, startPose, endPose, containmentOBBs, centerline } = context;
    const w        = { ...DEFAULT_WEIGHTS, ...(config ?? {}) };
    const MAX_ITER = config?.maxIter ?? 50000;
    const obbs     = containmentOBBs ?? [];
    const BATCH    = 500;

    const { points, ceilingHeight } = centerline;
    const halfCeil = ceilingHeight / 2;
    const n = points.length;
    function poseAt(pt) {
      return { x: pt[0], y: pt[1] + halfCeil, z: pt[2], yaw: 0, pitch: 0, roll: 0 };
    }
    const stairBasePose = poseAt(points[Math.min(1, n - 1)]);
    const stairMidPt    = points[Math.floor(n / 2)];
    const midPose       = poseAt(stairMidPt);
    const stairTopPose  = poseAt(points[Math.max(n - 2, 0)]);

    let bxMin = Infinity, bxMax = -Infinity;
    let byMin = Infinity, byMax = -Infinity;
    let bzMin = Infinity, bzMax = -Infinity;
    for (const quad of collisionQuads) {
      for (const [x, y, z] of quad.vertices) {
        if (x < bxMin) bxMin = x; if (x > bxMax) bxMax = x;
        if (y < byMin) byMin = y; if (y > byMax) byMax = y;
        if (z < bzMin) bzMin = z; if (z > bzMax) bzMax = z;
      }
    }
    function clampPose(p) {
      return {
        ...p,
        x: Math.max(bxMin, Math.min(bxMax, p.x)),
        y: Math.max(byMin, Math.min(byMax, p.y)),
        z: Math.max(bzMin, Math.min(bzMax, p.z)),
      };
    }

    let poses   = [startPose, stairBasePose, midPose, stairTopPose, endPose];
    let segData = [
      evalSegment(startPose,    stairBasePose, collisionQuads, halfExtents, obbs),
      evalSegment(stairBasePose, midPose,      collisionQuads, halfExtents, obbs),
      evalSegment(midPose,      stairTopPose,  collisionQuads, halfExtents, obbs),
      evalSegment(stairTopPose,  endPose,       collisionQuads, halfExtents, obbs),
    ];
    let energy = totalEnergy(segData, poses, w);

    let bestPoses   = poses.map(p => ({ ...p }));
    let bestSegData = segData.map(s => ({ ...s }));
    let bestEnergy  = energy;

    const T_START = 10.0, T_END = 0.001;
    const COOL    = Math.pow(T_END / T_START, 1 / Math.max(MAX_ITER, 1));
    let T = T_START;

    for (let iter = 0; iter < MAX_ITER; iter++) {
      if (shouldCancel?.()) break;
      T *= COOL;

      const r = Math.random();
      let newPoses, newSegData, newEnergy;

      if (r < 0.55 && poses.length > 2) {
        const i   = 1 + Math.floor(Math.random() * (poses.length - 2));
        const dof = DOFS[Math.floor(Math.random() * 6)];
        newPoses     = poses.map(p => ({ ...p }));
        newPoses[i]  = clampPose({ ...poses[i], [dof]: poses[i][dof] + randn() * SIGMA[dof] * T });
        newSegData   = segData.slice();
        newSegData[i - 1] = evalSegment(newPoses[i - 1], newPoses[i],     collisionQuads, halfExtents, obbs);
        newSegData[i]     = evalSegment(newPoses[i],     newPoses[i + 1], collisionQuads, halfExtents, obbs);
        newEnergy = totalEnergy(newSegData, newPoses, w);

      } else if (r < 0.65 && poses.length > 2) {
        const i     = 1 + Math.floor(Math.random() * (poses.length - 2));
        const dof   = ROTATION_DOFS[Math.floor(Math.random() * 3)];
        const delta = randn() * SIGMA[dof] * T;
        newPoses    = applyRotationPropagation(poses, i, poses.length - 1, dof, delta);
        newSegData  = segData.slice();
        const lastInterior = poses.length - 2;
        for (let j = i - 1; j <= lastInterior; j++) {
          newSegData[j] = evalSegment(newPoses[j], newPoses[j + 1], collisionQuads, halfExtents, obbs);
        }
        newEnergy = totalEnergy(newSegData, newPoses, w);

      } else if (r < 0.75 && poses.length > 2) {
        const i     = 1 + Math.floor(Math.random() * (poses.length - 2));
        const dof   = ROTATION_DOFS[Math.floor(Math.random() * 3)];
        const delta = randn() * SIGMA[dof] * T;
        newPoses    = applyRotationPropagation(poses, 1, i + 1, dof, delta);
        newSegData  = segData.slice();
        for (let j = 0; j <= i; j++) {
          newSegData[j] = evalSegment(newPoses[j], newPoses[j + 1], collisionQuads, halfExtents, obbs);
        }
        newEnergy = totalEnergy(newSegData, newPoses, w);

      } else if (r < 0.90 || poses.length <= 2) {
        let worstIdx = 0;
        for (let i = 1; i < segData.length; i++) {
          const ci = segData[i].collEnergy, c0 = segData[worstIdx].collEnergy;
          if (ci > c0 || (ci === c0 && segData[i].duration > segData[worstIdx].duration)) worstIdx = i;
        }
        let mid = lerpPose(poses[worstIdx], poses[worstIdx + 1], 0.5);
        for (const dof of DOFS) mid[dof] += randn() * SIGMA[dof] * T * 0.5;
        mid = clampPose(mid);
        newPoses = [
          ...poses.slice(0, worstIdx + 1),
          mid,
          ...poses.slice(worstIdx + 1),
        ];
        newSegData = [
          ...segData.slice(0, worstIdx),
          evalSegment(newPoses[worstIdx],     mid,                    collisionQuads, halfExtents, obbs),
          evalSegment(mid,                    newPoses[worstIdx + 2], collisionQuads, halfExtents, obbs),
          ...segData.slice(worstIdx + 1),
        ];
        newEnergy = totalEnergy(newSegData, newPoses, w);

      } else {
        const i    = 1 + Math.floor(Math.random() * (poses.length - 2));
        newPoses   = [...poses.slice(0, i), ...poses.slice(i + 1)];
        newSegData = [
          ...segData.slice(0, i - 1),
          evalSegment(newPoses[i - 1], newPoses[i], collisionQuads, halfExtents, obbs),
          ...segData.slice(i + 1),
        ];
        newEnergy = totalEnergy(newSegData, newPoses, w);
      }

      const dE = newEnergy - energy;
      if (dE < 0 || Math.random() < Math.exp(-dE / Math.max(T, 1e-10))) {
        poses   = newPoses;
        segData = newSegData;
        energy  = newEnergy;
        if (energy < bestEnergy) {
          bestEnergy  = energy;
          bestPoses   = poses.map(p => ({ ...p }));
          bestSegData = segData.map(s => ({ ...s }));
        }
      }

      if ((iter + 1) % BATCH === 0) {
        onProgress?.({ poses: bestPoses, energy: bestEnergy, temperature: T,
                       iteration: iter + 1, maxIter: MAX_ITER });
        await new Promise(resolve => setTimeout(resolve, 0));
      }
    }

    const fits = bestSegData.every(s => s.collEnergy === 0);
    let tightestIndex = 0, minC = Infinity;
    for (let i = 0; i < bestPoses.length; i++) {
      const { minClearance } = checkCollisions(computeOBB(bestPoses[i], halfExtents), collisionQuads);
      if (minClearance < minC) { minC = minClearance; tightestIndex = i; }
    }

    return { poses: bestPoses, fits, tightestIndex };
  },

  formatProgress(data, container) {
    const { temperature, iteration, maxIter = 50000 } = data;
    if (!container.querySelector('#tl-sa-temp')) {
      container.innerHTML = `
        <span id="tl-sa-temp" style="font-family:monospace;font-size:12px;color:#64ffda;">T=5.000</span>
        <span id="tl-sa-iter" style="font-family:monospace;font-size:12px;color:#aaa;">
          0 / ${maxIter.toLocaleString()}
        </span>
        <div style="flex:1;height:6px;background:#1a2a3a;border-radius:3px;overflow:hidden;">
          <div id="tl-sa-bar" style="height:100%;width:0%;background:#64ffda;border-radius:3px;
            transition:width 0.4s;"></div>
        </div>`;
    }
    const t = container.querySelector('#tl-sa-temp');
    const i = container.querySelector('#tl-sa-iter');
    const b = container.querySelector('#tl-sa-bar');
    if (t) t.textContent = `T=${temperature.toFixed(3)}`;
    if (i) i.textContent = `${iteration.toLocaleString()} / ${maxIter.toLocaleString()}`;
    if (b) b.style.width = `${Math.min(100, (iteration / maxIter) * 100)}%`;
  },
};
```

- [ ] **Step 4: Run test to verify it passes**

```bash
npx vitest run src/solver/planners/sa.test.js
```

Expected: PASS — all 7 tests green.

- [ ] **Step 5: Run all tests**

```bash
npm test
```

Expected: All tests pass (`trajectory.test.js` still tests the old `optimizeTrajectory` — that's fine, it hasn't been removed yet).

- [ ] **Step 6: Commit**

```bash
git add src/solver/planners/sa.js src/solver/planners/sa.test.js
git commit -m "feat: add SA planner implementing planner interface"
```

---

## Task 4: Refactor `worker.js` to generic planner worker

**Files:**
- Modify: `src/solver/worker.js`

The old `worker.js` hardcodes `optimizeTrajectory`. Replace it entirely.

- [ ] **Step 1: Replace `src/solver/worker.js`**

```js
import { saPlanner }          from './planners/sa.js';
import { buildPlannerContext } from './context.js';
import { buildTrajectory }     from './trajectory.js';

const PLANNERS = { sa: saPlanner };
let cancelFlag = false;

self.onmessage = async ({ data }) => {
  if (data.type === 'cancel') { cancelFlag = true; return; }
  if (data.type === 'start') {
    cancelFlag = false;
    const { stairwellParams, boxDims, plannerType = 'sa', plannerConfig } = data;
    try {
      const planner = PLANNERS[plannerType];
      if (!planner) throw new Error(`Unknown planner: ${plannerType}`);
      const context = buildPlannerContext(stairwellParams, boxDims);
      const result  = await planner.plan(
        context, plannerConfig,
        (d) => self.postMessage({ type: 'progress', plannerType, ...d }),
        () => cancelFlag,
      );
      const { segmentTimes, totalTime } = buildTrajectory(result.poses);
      self.postMessage({
        type: cancelFlag ? 'canceled' : 'done',
        poses: result.poses, segmentTimes, totalTime,
        fits: result.fits ?? false,
        tightestIndex: result.tightestIndex ?? 0,
      });
    } catch (err) {
      self.postMessage({ type: 'error', message: err.message });
    }
  }
};
```

- [ ] **Step 2: Run all tests**

```bash
npm test
```

Expected: All tests pass (the worker isn't directly unit-tested; existing tests for other modules still pass).

- [ ] **Step 3: Commit**

```bash
git add src/solver/worker.js
git commit -m "refactor: worker.js becomes generic planner worker with registry"
```

---

## Task 5: Replace `trajectory.js` with `buildTrajectory` only

**Files:**
- Modify: `src/solver/trajectory.js`
- Modify: `src/solver/trajectory.test.js`

The old SA code in `trajectory.js` now lives in `planners/sa.js` and is no longer needed here. Replace the file contents.

- [ ] **Step 1: Replace `src/solver/trajectory.test.js`**

```js
import { describe, it, expect } from 'vitest';
import { buildTrajectory } from './trajectory.js';

describe('buildTrajectory', () => {
  it('returns segmentTimes with length poses.length - 1', () => {
    const poses = [
      { x: 0, y: 1, z: -3, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  0, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  3, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes } = buildTrajectory(poses);
    expect(segmentTimes).toHaveLength(2);
  });

  it('totalTime equals sum of segmentTimes', () => {
    const poses = [
      { x: 0, y: 1, z: -3, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  0, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  3, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes, totalTime } = buildTrajectory(poses);
    expect(totalTime).toBeCloseTo(segmentTimes.reduce((a, b) => a + b, 0), 10);
  });

  it('each segment duration is positive', () => {
    const poses = [
      { x: 0, y: 1, z: 0, yaw: 0, pitch: 0, roll: 0 },
      { x: 1, y: 1, z: 0, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes } = buildTrajectory(poses);
    expect(segmentTimes[0]).toBeGreaterThan(0);
  });

  it('returns the same poses reference', () => {
    const poses = [
      { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 },
      { x: 1, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { poses: result } = buildTrajectory(poses);
    expect(result).toBe(poses);
  });
});
```

- [ ] **Step 2: Run test to verify it fails**

```bash
npx vitest run src/solver/trajectory.test.js
```

Expected: FAIL — `buildTrajectory is not exported` (old trajectory.js doesn't export it yet).

- [ ] **Step 3: Replace `src/solver/trajectory.js`**

```js
import { segmentDuration } from './utils.js';

export function buildTrajectory(poses) {
  const segmentTimes = poses.slice(0, -1).map((p, i) => segmentDuration(p, poses[i + 1]));
  const totalTime    = segmentTimes.reduce((a, b) => a + b, 0);
  return { poses, segmentTimes, totalTime };
}
```

- [ ] **Step 4: Run all tests**

```bash
npm test
```

Expected: All tests pass. `trajectory.test.js` now tests `buildTrajectory`. Previous SA tests now live exclusively in `planners/sa.test.js`.

- [ ] **Step 5: Commit**

```bash
git add src/solver/trajectory.js src/solver/trajectory.test.js
git commit -m "refactor: trajectory.js reduced to buildTrajectory timing layer"
```

---

## Task 6: Update `timeline.js` progress rendering

**Files:**
- Modify: `src/ui/timeline.js`

`renderSolving()` currently hardcodes SA-specific DOM. Replace the progress area with a generic placeholder `<div id="tl-planner-progress">`. Change `updateProgress` to accept a `formatter` argument and delegate rendering to it.

- [ ] **Step 1: Update `renderSolving()` in `src/ui/timeline.js`**

Replace the current `renderSolving` function:

```js
function renderSolving() {
  container.innerHTML = `
    <div style="display:flex;align-items:center;gap:12px;height:100%;padding:0 16px;">
      <button id="tl-cancel" style="color:#ff4444;border:1px solid #ff4444;
        border-radius:4px;padding:4px 12px;background:transparent;cursor:pointer;">
        ✕ Cancel
      </button>
      <div id="tl-planner-progress" style="display:flex;flex:1;align-items:center;gap:12px;"></div>
    </div>`;
  container.querySelector('#tl-cancel').addEventListener('click', onCancel);
}
```

- [ ] **Step 2: Update `updateProgress` in `src/ui/timeline.js`**

Replace the current `updateProgress` function:

```js
function updateProgress(data, formatter) {
  const progressEl = container.querySelector('#tl-planner-progress');
  if (progressEl && formatter) formatter(data, progressEl);
}
```

Update the return value at the bottom of `createTimeline` to match the new signature:

```js
return { setState, updateProgress, setResult, updatePlayhead, setPlayState };
```

(This line is unchanged — `updateProgress` is already in the return object; the signature change is transparent to callers.)

- [ ] **Step 3: Run all tests**

```bash
npm test
```

Expected: All tests pass.

- [ ] **Step 4: Commit**

```bash
git add src/ui/timeline.js
git commit -m "refactor: timeline progress rendering delegates to planner formatter"
```

---

## Task 7: Update `main.js`

**Files:**
- Modify: `src/main.js`

Two changes: `lerpPose` import already updated in Task 1. Now add the `saPlanner` import and update the worker progress handler.

- [ ] **Step 1: Add `saPlanner` import to `src/main.js`**

After the existing imports, add:

```js
import { saPlanner } from './solver/planners/sa.js';
```

The `lerpPose` import should already read `from './solver/utils.js'` (done in Task 1).

- [ ] **Step 2: Update the worker progress handler in `src/main.js`**

Find the `currentWorker.onmessage` handler. Change the `'progress'` branch from:

```js
if (data.type === 'progress') {
  timeline.updateProgress(data);
  renderGhostTrail(data.poses);
}
```

to:

```js
if (data.type === 'progress') {
  timeline.updateProgress(data, saPlanner.formatProgress);
  renderGhostTrail(data.poses);
}
```

- [ ] **Step 3: Run all tests**

```bash
npm test
```

Expected: All tests pass.

- [ ] **Step 4: Start the dev server and verify solve still works**

```bash
npm run dev
```

Open [http://localhost:5173](http://localhost:5173). Click **▶ SOLVE**. Verify:
- The progress bar and temperature readout appear while solving
- Ghost trail updates during solve
- "✓ Box fits!" or best-trajectory banner appears when done
- Playback scrubber works

- [ ] **Step 5: Commit**

```bash
git add src/main.js
git commit -m "refactor: main.js uses saPlanner.formatProgress for timeline progress"
```

---

## Self-Review Checklist

**Spec coverage:**
- ✅ `utils.js` with all shared pose math — Task 1
- ✅ `context.js` with `buildPlannerContext` + start/end pose derivation moved out — Task 2
- ✅ `planners/sa.js` with `{ plan, formatProgress, evalSegment }` — Task 3
- ✅ `trajectory.js` reduced to `buildTrajectory` — Task 5
- ✅ `worker.js` generic with planner registry, `plannerType`/`plannerConfig` fields — Task 4
- ✅ `timeline.js` placeholder + `updateProgress(data, formatter)` — Task 6
- ✅ `main.js` imports updated — Tasks 1, 7
- ✅ Tests reorganized to follow new module boundaries — Tasks 1–5

**Placeholder scan:** No TBDs or incomplete steps found.

**Type consistency:**
- `updateProgress(data, formatter)` — defined in Task 6, called in Task 7 ✅
- `saPlanner.formatProgress` — exported from `planners/sa.js` (Task 3), imported in `main.js` (Task 7) ✅
- `buildTrajectory` — defined in Task 5, imported by `worker.js` (Task 4) ✅
- `buildPlannerContext` — defined in Task 2, imported by `worker.js` (Task 4) ✅
- `evalSegment` — exported from `planners/sa.js` (Task 3), tested in `planners/sa.test.js` (Task 3) ✅
- `lerpPose` import in `main.js` updated in Task 1 before it's removed from `trajectory.js` in Task 5 ✅
