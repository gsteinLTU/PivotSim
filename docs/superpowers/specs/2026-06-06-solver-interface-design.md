# Solver Interface Refactor — Design Spec

**Date:** 2026-06-06
**Status:** Approved

## Problem

The solver is tightly coupled to the simulated-annealing (SA) implementation. Swapping in an alternative planner (e.g. RRT) requires touching the worker, the timeline, and the main entry point. Additionally, shared pose math (`lerpPose`, `euclideanDelta`, etc.) lives inside `trajectory.js` alongside SA-specific logic, making it awkward to reuse.

## Goals

- Define a planner interface that any algorithm can implement
- Decouple the worker message protocol from the SA algorithm
- Separate path planning (find collision-free poses) from trajectory building (add timing)
- Extract shared pose math into a reusable `utils.js`
- Allow each planner to own its own progress UI

## Non-goals

- Implementing RRT or any new planner (this spec covers the interface only)
- Changing stairwell geometry, collision detection, or playback logic

---

## File Structure

```
src/solver/
  utils.js              — shared pose math and speed constants (new)
  collision.js          — unchanged
  path.js               — unchanged
  context.js            — buildPlannerContext() (new)
  trajectory.js         — buildTrajectory(poses) only; SA internals removed (refactored)
  worker.js             — generic worker with planner registry (refactored)
  planners/
    sa.js               — SA planner: { plan, formatProgress } (new, extracted from trajectory.js)
```

---

## Planner Interface

A planner module must export:

```js
{
  plan(context, config, onProgress, shouldCancel): Promise<PlanResult>
  formatProgress(data, container): void
}
```

### PlanResult

```js
{
  poses: [{ x, y, z, yaw, pitch, roll }, ...],  // required — minimum contract
  fits?: boolean,                                // optional
  tightestIndex?: number,                        // optional
}
```

`poses` is the only required field. `fits` and `tightestIndex` are computed inside the planner when it has the information cheaply; they need not be recomputed by the caller.

### PlannerContext

```js
{
  collisionQuads,    // collision geometry from buildStairwell
  halfExtents,       // [hW, hH, hL]
  startPose,         // { x, y, z, yaw, pitch, roll } — derived from centerline
  endPose,           // { x, y, z, yaw, pitch, roll } — derived from centerline
  containmentOBBs,   // OBB[] — empty array if none
  centerline,        // { points, totalLength, ceilingHeight } — seeding hint; planners may ignore
}
```

### PlannerConfig

Planner-specific. Passed through from the worker's `start` message as `plannerConfig`. Each planner documents its own config shape.

---

## Module Designs

### `utils.js`

Exports shared pose math extracted from `trajectory.js`:

```js
export const MAX_LINEAR_SPEED  = 0.5;   // m/s
export const MAX_ANGULAR_SPEED = 0.5;   // rad/s

export function euclideanDelta(a, b)
export function angularDelta(a, b)
export function segmentDuration(a, b)
export function lerpPose(a, b, t)
export function applyRotationPropagation(poses, startIdx, endIdx, dof, delta)
```

`main.js` updates its `lerpPose` import from `./solver/trajectory.js` to `./solver/utils.js`.

---

### `context.js`

```js
export function buildPlannerContext(stairwellParams, boxDims): PlannerContext
```

Consolidates all problem-setup work currently split between `worker.js` and the top of `optimizeTrajectory`:

1. Calls `buildStairwell`, `getHalfExtents`, `buildCenterline`, `buildContainmentOBBs`
2. Derives `startPose` and `endPose` from the centerline (logic moved out of `optimizeTrajectory`):
   - `startPose` = midpoint between `points[0]` and `points[1]`, vertically centered in hallway
   - `endPose` = midpoint between `points[n-1]` and `points[n-2]`, vertically centered

---

### `trajectory.js` (refactored)

Repurposed as a thin timing layer. Only job: add `segmentTimes` and `totalTime` to a raw path.

```js
import { segmentDuration } from './utils.js';

export function buildTrajectory(poses) {
  const segmentTimes = poses.slice(0, -1).map((p, i) => segmentDuration(p, poses[i + 1]));
  const totalTime    = segmentTimes.reduce((a, b) => a + b, 0);
  return { poses, segmentTimes, totalTime };
}
```

This is where a different speed model would be applied in future if needed.

---

### `worker.js` (refactored)

Generic worker — owns message protocol, delegates to a planner from the registry.

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

Adding a new planner = one import + one registry entry.

---

### `planners/sa.js` (new)

Contains all SA internals currently in `trajectory.js`:
- Private: `evalSegment`, `computeOBB`, `totalEnergy`, `randn`, `SIGMA`, the annealing loop
- Exported: `DEFAULT_WEIGHTS`
- Interface implementation:

```js
export const saPlanner = {
  async plan(context, config, onProgress, shouldCancel) → { poses, fits, tightestIndex },
  formatProgress(data, container) → void,
};
```

`evalSegment` remains exported from this module (used in tests).

`plan` is `optimizeTrajectory` adapted to receive `context.startPose` and `context.endPose` directly instead of deriving them internally. The centerline-based keypoint seeding (stair junctions, midpoint) remains — it uses `context.centerline`.

`formatProgress` renders the SA-specific progress UI (temperature readout, iteration counter, progress bar) into `container`. The hardcoded `50,000` in `timeline.js` moves here.

---

### `timeline.js` (updated)

`renderSolving()` renders the cancel button plus a `<div id="tl-planner-progress">` placeholder for planner-specific content.

`updateProgress(data, formatter)` calls `formatter(data, container.querySelector('#tl-planner-progress'))`. Callers pass the active planner's `formatProgress` function.

`main.js` imports `saPlanner` directly and passes `saPlanner.formatProgress` to `timeline.updateProgress`.

---

## Data Flow

```
main.js
  └─ worker.postMessage({ type:'start', stairwellParams, boxDims, plannerType:'sa', plannerConfig })
       └─ worker.js
            ├─ buildPlannerContext(stairwellParams, boxDims)  → context
            ├─ PLANNERS['sa'].plan(context, config, onProgress, shouldCancel)  → { poses, fits, tightestIndex }
            ├─ buildTrajectory(poses)  → { poses, segmentTimes, totalTime }
            └─ postMessage({ type:'done', poses, segmentTimes, totalTime, fits, tightestIndex })

main.js (progress)
  └─ saPlanner.formatProgress(data, progressContainer)
```

---

## Testing

| File | Coverage |
|------|----------|
| `utils.test.js` (new) | `euclideanDelta`, `angularDelta`, `segmentDuration`, `lerpPose`, `applyRotationPropagation` — moved from `trajectory.test.js` |
| `context.test.js` (new) | `buildPlannerContext` returns correct `startPose`/`endPose` shape |
| `planners/sa.test.js` (new) | `plan()` tests adapted from `optimizeTrajectory` tests; `evalSegment` tests co-located |
| `trajectory.test.js` (refactored) | `buildTrajectory` only |
| `collision.test.js`, `path.test.js` | unchanged |

No new test surface; this is a reorganization that preserves existing coverage.
