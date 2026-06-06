# PivotSim Phase 2 — Trajectory Solver + Timeline

## Goal

Add an automated trajectory solver that determines whether a box can navigate a stairwell, and if so, how. A simulated annealing optimizer searches for a collision-free 6-DOF trajectory. A timeline UI lets the user watch the solver run live and play back the result.

## Context

Builds on Phase 1.5 (box model + SAT collision detection). The stairwell geometry, collision quad format, OBB computation, and `checkCollisions` API are unchanged except for the signed clearance extension noted below.

---

## New Files

| File | Purpose |
|------|---------|
| `src/solver/path.js` | Centerline polyline + arc-length parameterization |
| `src/solver/trajectory.js` | SA energy function + variable-length optimizer |
| `src/solver/worker.js` | Web Worker wrapper — runs SA off the main thread |
| `src/ui/timeline.js` | Timeline bar: idle → solving → done states |

**Modified files:** `src/main.js` (Solve wiring, result banner, ghost trail), `index.html` (timeline height, result banner div), `src/solver/collision.js` (signed clearance), `src/ui/config-panel.js` (lock/unlock API).

---

## Time Model

Time is **derived from geometry**, never stored on poses. Given constants:

```js
MAX_LINEAR_SPEED  = 0.5   // m/s
MAX_ANGULAR_SPEED = 0.5   // rad/s (~30°/s)
```

The duration of a segment between adjacent poses is:

```js
segmentDuration(a, b) = Math.max(
  euclideanDelta(a, b) / MAX_LINEAR_SPEED,
  angularDelta(a, b)   / MAX_ANGULAR_SPEED
)
```

Total trajectory time = sum of all segment durations. This is used for:
- **Playback**: scrubber maps to real seconds; tight corners with lots of rotation get more scrubber space than long straight slides.
- **Between-pose sampling density**: K interpolated collision checks per segment is proportional to `segmentDuration` (longer/slower segments get more checks).
- **Energy**: a `w_time × totalTime` term discourages unnecessarily slow or winding paths.

---

## Module: `src/solver/path.js`

### Purpose

Compute a centerline polyline through the stairwell. Provides the two initial poses that seed the SA (start and end of the path).

### API

```js
export function buildCenterline(params)
// Returns: { points: [[x,y,z], ...], totalLength: number }
// Points: bottom hallway far end → hallway/stair junction →
//         one point per step tread center →
//         stair/hallway junction → top hallway far end.

export function getEndpoints(centerline)
// Returns: { start: [x,y,z], end: [x,y,z] }
// The entry and exit points of the stairwell path.
// Used to build the two initial SA poses.
```

### Geometry

The centerline follows the floor at horizontal midpoint:

1. Bottom hallway: from the far end back to z=0, at y=0, x=0
2. Stair flight: one point per step at `(0, i×rise + rise/2, i×run + run/2)`
3. Top hallway: from `(0, totalRise, totalRun)` to the far end

Turns are handled by following the rotated hallway group's local coordinate axes (same transform as in `stairwell.js`).

### Tests

- `buildCenterline` returns at least `numSteps + 2` points
- `getEndpoints` returns `start` and `end` as length-3 arrays
- First point is in the bottom hallway (y ≈ 0, z < 0)
- Last point is in the top hallway (y ≈ totalRise)

---

## Module: `src/solver/trajectory.js`

### Purpose

Pure SA optimizer with a variable-length pose array. No Three.js dependency.

### Types

```js
// Pose — all angles in RADIANS
{ x, y, z, yaw, pitch, roll }

// Segment — derived, never stored
{ duration: number, kSamples: number }  // computed on demand

// TrajectoryResult
{
  poses: Pose[],          // variable length, ≥ 2
  segmentTimes: number[], // length poses.length - 1; duration of each segment
  totalTime: number,      // sum of segmentTimes
  fits: boolean,          // true if best energy has zero collision penalty
  tightestIndex: number,  // index of pose with minimum clearance
  finalEnergy: number,
}
```

### Energy Function

Computed over all poses and between-pose samples:

```
E = w_col  × Σ_segments  segmentCollisionEnergy(i, i+1)
  - w_clr  × Σ_segments  segmentClearanceReward(i, i+1)
  + w_rot  × Σ_segments  angularDelta(pose_i, pose_{i+1})
  + w_pos  × Σ_segments  euclideanDelta(pose_i, pose_{i+1})
  + w_time × totalTime
```

Default weights: `{ w_col: 100, w_clr: 1, w_rot: 0.1, w_pos: 0.5, w_time: 0.01 }`.

**`segmentCollisionEnergy(i, i+1)`**: sample K interpolated poses between pose_i and pose_{i+1}, where `K = Math.max(5, Math.ceil(segmentDuration(i, i+1) / 0.2))`. For each sample, compute signed clearance. Find the sample with the worst (most negative) clearance, then check several additional samples clustered around it (±10% of segment length) to better locate the true minimum. Sum of `max(0, -clearance)²` across all samples.

**`segmentClearanceReward(i, i+1)`**: sum of `min(clearance, CAP)` across the same samples (CAP = 0.3m). Rewards being away from walls.

**Signed clearance**: `checkCollisions` returns a signed `minClearance` — negative when penetrating (see `collision.js` changes). This is what `clearance_i` refers to throughout.

### SA Loop

Three move types, chosen randomly each iteration:

**Perturb** (70% of moves): pick a random pose index (excluding first and last, which are fixed at the start/end endpoints), perturb one random DOF by `Normal(0, σ × T)`:
```
σ = { x: 0.1, y: 0.1, z: 0.1, yaw: 0.3, pitch: 0.2, roll: 0.2 }
```

**Split** (20% of moves): find the segment with the highest `segmentCollisionEnergy`. Insert a new pose at the lerp midpoint of that segment (position and orientation), plus a small random perturbation. This is how the trajectory grows resolution at problem spots.

**Merge** (10% of moves): find a non-endpoint pose where removing it (and re-checking the combined segment from its predecessor to its successor) does not increase `segmentCollisionEnergy`. If found, remove it. Keeps the trajectory compact. If no mergeable pose exists, skip.

```
T_start = 5.0
T_end   = 0.001
maxIter = 50000
cooling = geometric: T *= (T_end / T_start)^(1 / maxIter) each iteration
```

Accept/reject via Metropolis criterion: accept if ΔE < 0, else accept with probability `exp(-ΔE / T)`.

Progress callback invoked every 500 iterations with current best state.

### Initial State

Two poses:
- `poses[0]`: position = centerline start, all angles = 0 (fixed, never perturbed)
- `poses[1]`: position = centerline end, all angles = 0 (fixed, never perturbed)

The direct path between them obviously collides — SA immediately wants to split it.

### API

```js
export function optimizeTrajectory(collisionQuads, halfExtents, centerline, weights, onProgress)
// collisionQuads: from buildStairwell
// halfExtents: [hWidth, hHeight, hLength] from getHalfExtents
// centerline: from buildCenterline (used for endpoint positions)
// weights: { w_col, w_clr, w_rot, w_pos, w_time } — uses defaults if omitted
// onProgress: ({ poses, segmentTimes, totalTime, energy, temperature, iteration }) => void
// Returns: TrajectoryResult
```

### Tests

- Returns `TrajectoryResult` with `poses.length >= 2`
- `segmentTimes.length === poses.length - 1`
- `totalTime` equals sum of `segmentTimes`
- `fits` is `false` when box is larger than stairwell in all dimensions
- Split move increases `poses.length` by 1
- Merge move decreases `poses.length` by 1 (when valid)
- With a trivially large stairwell and small box, `fits === true`
- `tightestIndex` is valid index into `poses`

---

## Module: `src/solver/worker.js`

Web Worker entry point. Imports `path.js`, `trajectory.js`, `collision.js`, `box.js`, `stairwell.js`.

### Message Protocol

**Incoming** (main → worker):
```js
{ type: 'start', stairwellParams, boxDims, weights }
{ type: 'cancel' }
```

**Outgoing** (worker → main):
```js
{ type: 'progress', poses, segmentTimes, totalTime, energy, temperature, iteration }
{ type: 'done', fits, poses, segmentTimes, totalTime, tightestIndex, finalEnergy }
{ type: 'canceled', poses, segmentTimes, totalTime }  // best result so far
```

### Behavior

On `start`:
1. `buildStairwell(stairwellParams)` → `collisionQuads`
2. `buildCenterline(stairwellParams)` → `centerline`
3. `getHalfExtents(boxDims)` → `halfExtents`
4. `optimizeTrajectory(collisionQuads, halfExtents, centerline, weights, onProgress)`
5. Post `done`

On `cancel`: sets a cancellation flag. SA checks it every progress callback; posts `canceled` with best-so-far result and exits.

---

## Module: `src/ui/timeline.js`

### States

**idle**:
```
[ ▶ SOLVE                                                              ]
```

**solving**:
```
[ ✕ Cancel    T=2.41   iter 12400 / 50000   [energy bar ████░░░░░░]  ]
```

**done**:
```
[ ↺  ⏮  ▶  [scrubber ●━━━━◆━━━━━━◆━━━━━━━━]  0:04 / 0:18   1× ▼ ]
```

Scrubber position maps to **real time** (seconds). Keyframe dots:
- Red: tightest clearance point (`tightestIndex`)
- Orange: any pose with clearance < 5cm

Time display shows `elapsed / totalTime` in `m:ss` format.

Result banner above the controls:
- `✓ Box fits!` — green, `fits === true`
- `~ Best trajectory found — may still collide` — orange, `fits === false`, solved
- `~ Canceled — partial result` — grey, canceled

### API

```js
export function createTimeline(container, callbacks)
// callbacks: {
//   onSolve: () => void,
//   onCancel: () => void,
//   onPlayheadChange: (seconds: number) => void,
// }
// Returns: {
//   setState(state: 'idle' | 'solving' | 'done', data?),
//   updateProgress({ energy, temperature, iteration, maxIter }),
//   setResult({ fits, tightestIndex, poses, segmentTimes, totalTime }),
//   updatePlayhead(seconds: number),
// }
```

`onPlayheadChange` receives **seconds** (not t ∈ [0,1]). `main.js` converts seconds to a pose index using `segmentTimes`.

### Tests

- Returns object with `setState`, `updateProgress`, `setResult`, `updatePlayhead`
- After `setState('idle')`: button matching `/solve/i` present
- After `setState('solving')`: cancel button present
- After `setState('done', { fits: true, ... })`: text matching `/fits/i` present
- After `setState('done', { fits: false, ... })`: no `✓` in content
- `onSolve` fires on solve button click
- `onCancel` fires on cancel button click

---

## `src/ui/config-panel.js` Changes

Add `lock()` and `unlock()` to the returned object:

```js
lock() {
  container.querySelectorAll('input, select').forEach(el => { el.disabled = true; });
},
unlock() {
  container.querySelectorAll('input, select').forEach(el => { el.disabled = false; });
},
```

---

## `src/solver/collision.js` Changes

**`testOBBvsQuad`**: change `return { collides: true, clearance: 0 }` to `return { collides: true, clearance: maxGap }`. When all axes show overlap, `maxGap` is negative — its magnitude is the penetration depth on the least-overlapping axis.

**`checkCollisions`**: return `minClearance` as the minimum `result.clearance` across **all** quads (not clamped — can be negative when any quad is penetrating). Remove the `collides ? 0 : minClearance` guard.

**Tests**: add a test verifying `minClearance < 0` when the box clearly penetrates a surface.

---

## `index.html` Changes

- Remove placeholder text from `#timeline`
- Add `<div id="result-banner"></div>` as first child of `#timeline`
- Increase `#timeline` grid row height from `60px` to `80px`

---

## `main.js` Changes

### Radians note

Worker-produced poses are in **radians**. Pass directly to `updateBoxMeshPose` — do **not** apply `poseRad()`.

### New state variables

```js
let currentWorker = null;
let currentTrajectory = null;    // TrajectoryResult or null
let isPlaying = false;
let playheadSeconds = 0;
let playSpeed = 1.0;
let ghostMeshes = [];            // fixed pool of semi-transparent box meshes
```

### Solve flow

```js
function startSolve() {
  panel.lock();
  timeline.setState('solving');
  clearGhostTrail();

  currentWorker = new Worker(new URL('./solver/worker.js', import.meta.url), { type: 'module' });
  currentWorker.onmessage = ({ data }) => {
    if (data.type === 'progress') {
      timeline.updateProgress(data);
      renderGhostTrail(data.poses, data.segmentTimes);
    } else {
      // 'done' or 'canceled'
      panel.unlock();
      currentTrajectory = (data.poses?.length >= 2) ? data : null;
      timeline.setState('done', data);
    }
  };
  currentWorker.postMessage({ type: 'start',
    stairwellParams: panel.getParams(), boxDims: currentBoxDims });
}
```

### Ghost trail

Fixed pool of `MAX_GHOST = 20` semi-transparent box meshes (opacity=0.12). On each progress message, subsample `data.poses` down to ≤20 evenly-spaced indices and update the pool's positions/rotations. No GC pressure between progress messages.

### Playback

`onPlayheadChange(seconds)` converts seconds to a pose index:

```js
function poseAtTime(trajectory, seconds) {
  let elapsed = 0;
  for (let i = 0; i < trajectory.segmentTimes.length; i++) {
    const dt = trajectory.segmentTimes[i];
    if (elapsed + dt >= seconds || i === trajectory.segmentTimes.length - 1) {
      const t = Math.min(1, (seconds - elapsed) / dt);
      return lerpPose(trajectory.poses[i], trajectory.poses[i + 1], t);
    }
    elapsed += dt;
  }
  return trajectory.poses[trajectory.poses.length - 1];
}
```

`lerpPose`: linear interpolation of x/y/z, linear interpolation of angles (good enough for small steps; no full slerp needed).

Play/pause: each animation frame, if playing, advance `playheadSeconds += playSpeed * deltaTime`, clamp to `[0, totalTime]`, call `onPlayheadChange`.

### Config change → idle

When stairwell params or box dims change (and a trajectory exists): `timeline.setState('idle')`, `currentTrajectory = null`, clear ghost trail.

---

## Future Improvement

**Continuous swept-volume collision**: the between-pose sampling is an approximation. A proper swept-volume check (computing the convex hull of two adjacent OBBs and testing it against each quad) would give exact guarantees. This requires a significant rewrite of `collision.js` and is deferred.

---

## Verification

1. `npm test` — all tests pass
2. `npm run dev` — Solve button in timeline bar
3. Click Solve → params lock, solving state, ghost trail appears
4. Cancel mid-solve → params unlock, partial trajectory shown
5. Solve completes → result banner, playback controls, scrubber with keyframe dots positioned by real time
6. Play → box animates, scrubber advances in real time
7. Click keyframe dot → playhead jumps to tight spot
8. Change stairwell param → timeline resets to idle
9. Oversized box → `fits: false`, tightest point highlighted
