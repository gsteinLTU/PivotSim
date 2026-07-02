# Multi-Goal RRT-Connect: Corridor-Aligned End Poses

**Date:** 2026-07-02  
**Status:** Approved

## Problem

The planner currently hardcodes `yaw: 0` for both `startPose` and `endPose` in `context.js`. This has two failure modes:

1. **Turned hallways:** A box approaching or leaving a turned hallway at `yaw: 0` is misaligned with the corridor direction, so the planner wastes budget fighting a bad start/end orientation.
2. **Large objects:** A box that physically fits through the stairwell may only be able to arrive at the top rotated 180° from `yaw: 0`. The planner never finds a path despite a valid solution existing.

## Solution

Two coordinated changes:

1. **Corridor-aligned yaw** — compute natural yaw from hallway geometry instead of hardcoding `0`.
2. **Multi-root goal tree** — initialize `treeGoal` with N roots (one per candidate arrival orientation). The existing RRT-Connect bidirectional search is unchanged; it naturally finds whichever goal orientation is reachable first.

## Design

### 1. Corridor-aligned yaw (`context.js` + `path.js`)

Add `getCorridorYaws(centerline)` to `path.js`. It returns `{ startYaw, endYaw }` computed from the hallway direction vectors:

- `startYaw`: direction from bottom hallway far-end toward stair base, projected onto XZ → `atan2(dx, dz)`
- `endYaw`: direction from stair top toward top hallway far-end, projected onto XZ → `atan2(dx, dz)`

For a straight staircase (no turn) both are `0`, preserving current behavior.

`buildPlannerContext` uses `startYaw` for `startPose` (single pose, entry orientation is fixed) and `endYaw` as the base for candidate goal poses.

### 2. Multi-root goal tree (`rrt-connect.js`)

`newTree(rootPose)` is updated to `newTree(rootPoses)` accepting either a single pose object or an array. Each pose in the array becomes a separate root node with `parent: -1`.

All existing tree operations are unchanged:
- `nearest` scans all nodes including multiple roots.
- `connect` grows toward any node, stopping when it reaches any root.
- `pathToRoot` walks the parent chain until `parent === -1`, which correctly terminates at whichever root was reached.
- `extractPath` concatenates start-side path and goal-side path — the goal-side path now ends at one of the N goal orientations.

The planner constructs `treeGoal` as:

```js
const treeGoal = newTree(context.endPoses);
```

### 3. Context API (`context.js`)

`buildPlannerContext` replaces the single `endPose` return value with an `endPoses` array. It gains an optional third argument `goalYawOffsets` (default `[0, Math.PI]`), passed in from the worker alongside the box dimensions and stairwell params.

**Computing candidates:** For each offset in `goalYawOffsets`, construct a candidate pose at `{endPosition, yaw: endYaw + offset, pitch: 0, roll: 0}`.

**Filtering:** Each candidate is checked with `checkCollisions(computeOBBFromPose(candidate, halfExtents), collisionQuads)`. Only poses where `minClearance >= 0` are included. No repositioning is attempted — the position is the corridor midpoint, so a collision means the box genuinely doesn't fit at that orientation.

**Fallback:** If all candidates are filtered out, `endPoses` is an empty array. The planner's existing `fallback()` path handles this (the `validState` check on `endPose` already covers this case — it needs updating to check `endPoses.length > 0` instead).

**Return shape change:** The context object returns both `endPoses` (array, for the RRT planner) and keeps `endPose` (singular) as an alias for `endPoses[0]`, or — if all candidates were filtered out — a display-only fallback pose at `{endPosition, yaw: endYaw, pitch: 0, roll: 0}`. This preserves backward compatibility with the SA planner and its tests, which destructure `endPose` directly and don't need multi-goal support.

### 4. Planner early-exit guard

The existing early-exit in `rrtPlanner.plan`:

```js
if (!validState(startPose, ...) || !validState(endPose, ...)) return fallback();
```

Changes to:

```js
if (!validState(startPose, ...) || endPoses.length === 0) return fallback();
```

(`endPoses` are already filtered valid in context, so no re-check needed.)

## File Changes Summary

| File | Change |
|------|--------|
| `src/solver/path.js` | Add `getCorridorYaws(centerline)` |
| `src/solver/context.js` | Use corridor-aligned `startYaw`; compute and filter `endPoses` array; remove `endPose` singular |
| `src/solver/planners/rrt-connect.js` | `newTree` accepts array; add `goalYawOffsets` to `DEFAULTS`; update early-exit guard |
| `src/solver/worker.js` | Pass `goalYawOffsets` from job config to `buildPlannerContext` |
| `src/main.js` | Use `endPoses[0]` for ghost placement |

## What Does Not Change

- The RRT-Connect bidirectional alternation (Ta/Tb swap) is identical.
- `extend`, `connect`, `steer`, `nearest`, `shortcut`, `sampleConfig` — all unchanged.
- The SA planner is unaffected (it doesn't use `endPose` directly).
- Gateway pose precomputation is unaffected.
- No change to collision quad format, OBB format, or trajectory building.

## Testing

- Straight staircase, no turn: `endPoses` should contain two poses (`yaw: 0` and `yaw: π`), both valid. Behavior is at least as good as before; 180° arrival now reachable.
- Turned staircase (e.g. 90° right): `startPose.yaw` and base `endYaw` should reflect the corridor directions, not `0`.
- Very tight staircase where 180° pose collides: `endPoses` has one entry (only `yaw: corridorYaw`). Planner behaves as current single-goal solver.
- All poses collide: `endPoses` is empty, `fallback()` is returned.
