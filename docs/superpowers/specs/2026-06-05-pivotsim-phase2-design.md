# PivotSim Phase 2 — Trajectory Solver + Timeline

## Goal

Add an automated trajectory solver that determines whether a box can navigate a stairwell, and if so, how. A simulated annealing optimizer searches for a collision-free 6-DOF trajectory. A timeline UI lets the user watch the solver run live and play back the result.

## Context

Builds on Phase 1.5 (box model + SAT collision detection). The stairwell geometry, collision quad format, OBB computation, and `checkCollisions` API are unchanged.

---

## New Files

| File | Purpose |
|------|---------|
| `src/solver/path.js` | Centerline polyline + arc-length parameterization |
| `src/solver/trajectory.js` | SA energy function + optimizer |
| `src/solver/worker.js` | Web Worker wrapper — runs SA off the main thread |
| `src/ui/timeline.js` | Timeline bar: idle → solving → done states |

**Modified files:** `src/main.js` (Solve wiring, result banner, ghost trail), `index.html` (timeline div replacement, result banner div), `src/solver/collision.js` (signed clearance), `src/ui/config-panel.js` (lock/unlock API).

---

## Module: `src/solver/path.js`

### Purpose

Compute a centerline polyline through the stairwell and expose evenly-spaced sample positions. These seed the SA initial state.

### API

```js
export function buildCenterline(params)
// Returns: { points: [[x,y,z], ...], totalLength: number }
// Points: bottom hallway end → hallway/stair junction →
//         one point per step tread center →
//         stair/hallway junction → top hallway end.
// Arc-length parameterized: consecutive points are not necessarily
// equal spacing — caller uses sampleN for even spacing.

export function sampleN(centerline, n)
// Returns: Array of n evenly-spaced { position: [x,y,z], forward: [dx,dy,dz] }
// forward is the unit tangent direction at that point (direction of travel).
// Used as starting position guess for each SA pose.
```

### Geometry

The centerline follows the floor of each segment at horizontal midpoint (x=0 for straight hallways, centerline of the hallway width for turned hallways):

1. Bottom hallway: from the far end of the bottom hallway to z=0, at y=0
2. Stair flight: one point per step at (0, i×risePerStep + risePerStep/2, i×runPerStep + runPerStep/2) — center of each tread surface
3. Top hallway: from (0, totalRise, totalRun) to the far end of the top hallway

Turns are handled by following the rotated hallway group's local axes (same rotation matrix used in `stairwell.js`).

### Tests

- `buildCenterline` returns a point array with at least `numSteps + 2` entries
- `sampleN(centerline, 10)` returns exactly 10 objects with `position` and `forward` properties
- `forward` vectors are unit length
- First and last sample positions are near the hallway ends

---

## Module: `src/solver/trajectory.js`

### Purpose

Pure SA optimizer. No Three.js dependency. Takes stairwell collision quads, box half-extents, and N starting poses; returns an optimized trajectory.

### Types

```js
// Pose — all angles in RADIANS
{ x, y, z, yaw, pitch, roll }

// TrajectoryResult
{
  poses: Pose[],         // length N
  fits: boolean,         // true if best energy has zero collision penalty
  tightestIndex: number, // index of pose with minimum clearance
  finalEnergy: number,
}
```

### Energy Function

```
E = w_col  × Σ max(0, -clearance_i)²        // penetration depth squared
  - w_clr  × Σ min(clearance_i, CAP)         // clearance reward, capped at CAP=0.3m
  + w_rot  × Σ angularDelta(pose_i, pose_{i+1})
  + w_pos  × Σ euclideanDelta(pose_i, pose_{i+1})
```

Default weights: `{ w_col: 100, w_clr: 1, w_rot: 0.1, w_pos: 0.5 }`. Penetration depth squared gives a stronger gradient near walls than linear would.

`clearance_i` is a **signed** clearance value: positive when clear, negative when penetrating. This requires a small change to `collision.js` (see below): `testOBBvsQuad` should return `clearance: maxGap` (the actual value, not clamped to 0), and `checkCollisions` should return `minClearance` as the minimum across all quads (can be negative). Existing callers are unaffected — they already check `collides` before using `minClearance`.

`angularDelta(a, b)`: sum of absolute differences in yaw, pitch, roll (in radians). Simple and cheap; a proper quaternion geodesic is not needed here.

`euclideanDelta(a, b)`: Euclidean distance between `[a.x, a.y, a.z]` and `[b.x, b.y, b.z]`.

### SA Loop

```
T_start = 5.0
T_end   = 0.001
maxIter = 50000
cooling = geometric: T *= (T_end / T_start)^(1 / maxIter) each iteration

perturb step:
  pick random i in [0, N)
  pick random DOF in { x, y, z, yaw, pitch, roll }
  delta ~ Normal(0, sigma × T)  where sigma = { x:0.1, y:0.1, z:0.1, yaw:0.3, pitch:0.2, roll:0.2 }
  candidate = copy poses with pose[i][DOF] += delta
  ΔE = E(candidate) - E(current)
  if ΔE < 0 or rand() < exp(-ΔE / T): accept candidate
```

Progress callback invoked every 500 iterations with `{ poses, energy, temperature, iteration }`.

### API

```js
export function optimizeTrajectory(collisionQuads, halfExtents, initialPoses, weights, onProgress)
// collisionQuads: from buildStairwell
// halfExtents: [hWidth, hHeight, hLength] from getHalfExtents
// initialPoses: Pose[] at centerline positions (from sampleN), angles = 0
// weights: { w_col, w_clr, w_rot, w_pos } — uses defaults if omitted
// onProgress: (progress) => void, called every 500 iterations
// Returns: TrajectoryResult
```

### Tests

- Returns a `TrajectoryResult` with `poses.length === n`
- `fits` is `false` when box is larger than stairwell in all dimensions
- Energy decreases (or stays same) on average over the run (test with fixed seed)
- `tightestIndex` points to the pose with minimum clearance in the result
- With a trivially large stairwell and small box, `fits === true`

---

## Module: `src/solver/worker.js`

Web Worker entry point. Imports `path.js`, `trajectory.js`, `collision.js`, `box.js`.

### Message Protocol

**Incoming** (main → worker):
```js
{ type: 'start', stairwellParams, boxDims, n: 100, weights }
{ type: 'cancel' }
```

**Outgoing** (worker → main):
```js
{ type: 'progress', poses, energy, temperature, iteration }  // every 500 iters
{ type: 'done', fits, poses, tightestIndex, finalEnergy }
{ type: 'canceled' }
```

### Behavior

On `start`:
1. Call `buildStairwell(stairwellParams)` → `collisionQuads`
2. Call `buildCenterline(stairwellParams)` → centerline
3. Call `sampleN(centerline, n)` → initial positions
4. Build `initialPoses`: each sample's `position` as `{x,y,z}`, all angles = 0
5. Call `optimizeTrajectory(...)` with `onProgress` posting `progress` messages
6. Post `done` message

On `cancel`: sets a flag checked inside the SA loop; on next progress callback, posts `canceled` and exits.

No unit tests for `worker.js` itself (Web Worker environment not available in Vitest/jsdom). The SA logic is tested via `trajectory.test.js`.

---

## Module: `src/ui/timeline.js`

### States

The timeline bar has three states:

**idle** — shown when no solve result exists yet (including after stairwell/box params change):
```
[ ▶ SOLVE                                                              ]
```

**solving** — shown while worker is running:
```
[ ✕ Cancel    T=2.41   iter 12400 / 50000   [energy bar ████░░░░░░]  ]
```

**done** — shown after worker completes or is canceled:
```
[ ↺  ⏮  ▶  [scrubber ●━━━━◆━━━━━━◆━━━━━━━━]  Clear: 8cm   1× ▼ ]
```

Keyframe dots on scrubber:
- Red dot: tightest clearance point (`tightestIndex`)
- Orange dots: poses where clearance < 5cm

Result banner rendered above the timeline (inside `#timeline` container):
- `✓ Box fits!` — green, shown when `fits === true`
- `~ Best trajectory found — may still collide` — orange, shown when `fits === false` and solve completed
- `~ Canceled — partial result` — grey, shown when canceled

### API

```js
export function createTimeline(container, callbacks)
// callbacks: {
//   onSolve: () => void,
//   onCancel: () => void,
//   onPlayheadChange: (t: number) => void,  // t in [0, 1]
// }
// Returns: {
//   setState(state: 'idle' | 'solving' | 'done', data?),
//   updateProgress({ energy, temperature, iteration, maxIter, poses }),
//   setResult({ fits, tightestIndex, poses }),
//   updatePlayhead(t: number),   // called by main.js animation loop
// }
```

`setState('idle')`: resets to solve button. Called when stairwell or box dims change (result is stale).

`setState('solving')`: shows progress UI. Called immediately when Solve is clicked.

`setState('done', { fits, tightestIndex, poses })`: shows playback UI + result banner.

`updateProgress(...)`: updates temperature readout, iteration count, energy bar fraction.

`updatePlayhead(t)`: moves scrubber thumb to position `t`. Called from main.js animation loop.

### Tests

- `createTimeline` returns object with `setState`, `updateProgress`, `setResult`, `updatePlayhead`
- After `setState('idle')`, container has a button with text matching `/solve/i`
- After `setState('solving')`, container has a cancel button
- After `setState('done', { fits: true, ... })`, container includes text matching `/fits/i`
- After `setState('done', { fits: false, ... })`, container does not include text matching `/✓/`
- `onSolve` callback fires when solve button is clicked
- `onCancel` callback fires when cancel button is clicked

---

## `src/ui/config-panel.js` Changes

Add `lock()` and `unlock()` methods to the returned object. `lock()` disables all inputs inside the container (sets `input.disabled = true`, `select.disabled = true`). `unlock()` re-enables them. Used by `main.js` during solving.

```js
// Added to return object:
lock() {
  container.querySelectorAll('input, select').forEach(el => { el.disabled = true; });
},
unlock() {
  container.querySelectorAll('input, select').forEach(el => { el.disabled = false; });
},
```

## `src/solver/collision.js` Changes

In `testOBBvsQuad`: change `return { collides: true, clearance: 0 }` to `return { collides: true, clearance: maxGap }`. The `maxGap` value is negative when colliding (all axes show overlap), giving the signed penetration depth.

In `checkCollisions`: track `minClearance` as the minimum `result.clearance` across all quads (not just non-colliding ones). Remove the `collides ? 0 : minClearance` clamp — return the raw minimum, which will be negative if any quad is penetrating.

Update `checkCollisions` tests to verify that `minClearance` is negative when the box is penetrating a surface.

## `index.html` Changes

- `#timeline` div gets `id="timeline"` kept; remove the placeholder text "Timeline (Phase 2)"
- Add `<div id="result-banner" style="display:none"></div>` inside `#timeline` above the controls
- Timeline height increases from `60px` to `80px` to fit result banner + controls

---

## `main.js` Changes

### Radians note

Worker-produced poses are already in **radians**. `main.js` passes them directly to `updateBoxMeshPose` — do **not** apply `poseRad()`. The `poseRad()` helper is only for poses coming from the UI config panel (which stores degrees).

### New state variables
```js
let currentWorker = null;
let currentTrajectory = null;   // Pose[] in radians, or null
let isPlaying = false;
let playheadT = 0;
let playSpeed = 1.0;
let ghostMeshes = [];           // Three.js meshes for ghost trail
```

### Solve flow
```js
function startSolve() {
  timeline.setState('solving');
  lockConfigPanel(true);
  clearGhostTrail();

  currentWorker = new Worker(new URL('./solver/worker.js', import.meta.url), { type: 'module' });
  currentWorker.onmessage = handleWorkerMessage;
  currentWorker.postMessage({ type: 'start', stairwellParams: panel.getParams(),
                               boxDims: currentBoxDims, n: 100 });
}

function handleWorkerMessage({ data }) {
  if (data.type === 'progress') {
    timeline.updateProgress(data);
    renderGhostTrail(data.poses);  // faded box meshes along trajectory
  } else if (data.type === 'done' || data.type === 'canceled') {
    lockConfigPanel(false);
    currentTrajectory = data.poses ?? null;
    timeline.setState('done', data);
  }
}
```

### Ghost trail
During solving, render the current-best trajectory as N semi-transparent box meshes (opacity=0.15, no collision color coding). Reuse a fixed pool of N ghost meshes to avoid GC pressure. Only update positions/rotations on each progress message.

### Playback
```js
function onPlayheadChange(t) {
  playheadT = t;
  if (!currentTrajectory) return;
  const idx = Math.round(t * (currentTrajectory.length - 1));
  const pose = currentTrajectory[idx];
  updateBoxMeshPose(currentBox, pose);  // pose already in radians from worker
  updateBoxCollision();
  timeline.updatePlayhead(t);
}
```

Play/pause animates `playheadT` forward by `playSpeed / fps` each frame in the animation loop.

### Config change → idle
When stairwell params or box dims change (and a trajectory exists), call `timeline.setState('idle')` and clear `currentTrajectory`. The result is stale.

---

## Verification

1. `npm test` — all tests pass (new: path, trajectory, timeline tests)
2. `npm run dev` — Solve button appears in timeline bar
3. Click Solve → parameters lock, solving state shows, ghost trail renders in viewport
4. Cancel mid-solve → unlocks params, shows partial result
5. Solve completes → result banner, playback controls, scrubber with keyframe dots
6. Play → box animates along trajectory
7. Click keyframe dot → playhead jumps to tight spot
8. Change a stairwell param → timeline resets to idle (result cleared)
9. Test with box obviously too large → `fits: false`, tightest point highlighted
