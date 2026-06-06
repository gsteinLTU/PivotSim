# Propagate-Rotation SA Moves

**Date:** 2026-06-06
**Status:** Draft

## Problem

The trajectory optimizer's simulated-annealing loop currently has one perturb move: nudge a single interior keypoint on one DOF. This means any rotation introduced at keypoint `i` interpolates back toward 0° at its neighbors — creating a "V" in orientation space. A solution requiring sustained rotation (e.g., yaw a sofa 45° through the entire stair flight) demands that many adjacent keypoints simultaneously hold the same non-zero rotation. Single-keypoint perturbations can never build that plateau efficiently; every intermediate state is penalized.

## Solution

Add two new SA move types — **forward propagate** and **backward propagate** — that apply the same rotation delta to a contiguous run of interior keypoints in a single step.

### Forward propagate

Pick a random interior keypoint index `i` and DOF. Apply `Δθ` to `poses[i]` through `poses[n-2]` (all interior keypoints from `i` to the last one before the end endpoint). The relative orientations between keypoints after `i` are unchanged; only the absolute orientation of that whole suffix shifts.

Effect: the rotation "enters" at keypoint `i`; the un-pivot is compressed into the single segment from the last interior keypoint to the fixed end endpoint.

### Backward propagate

Same, but applies `Δθ` to `poses[1]` through `poses[i]`. Shifts the orientation of all interior keypoints up to and including `i`.

Effect: the rotation "exits" at keypoint `i`; the pivot-in is compressed into the single segment from the fixed start endpoint to keypoint 1.

### Together

A forward propagate followed (in a later SA step) by a backward propagate can bracket a rotation plateau: the box pivots in early, maintains angle through the tight section, and un-pivots at the end. Neither move individually crosses a high-energy barrier because both preserve relative orientations within the run they shift.

## Scope

- Changes are confined to the `optimizeTrajectory` function in `src/solver/trajectory.js`.
- No new keypoint types. All keypoints remain plain `{x, y, z, yaw, pitch, roll}` pose objects.
- Splits continue to interpolate fresh from their neighbors' current pose values — no special casing needed.
- Only rotation DOFs (`yaw`, `pitch`, `roll`) are candidates for propagate moves. Propagating position deltas provides no benefit over single-keypoint position perturbation.

## Move probability budget

Current budget (approximate):
- Point perturb: 70%
- Split: 20%
- Merge: 10%

Proposed:
- Point perturb: 55%
- Forward propagate: 10%
- Backward propagate: 10%
- Split: 15%
- Merge: 10%

These are starting values; they may need tuning after observation.

## Affected files

| File | Change |
|---|---|
| `src/solver/trajectory.js` | Add forward/backward propagate branches inside the SA loop; update probability thresholds |

## Out of scope

- Stair-angle pitch seeding (initializing keypoints at `arctan(rise/run)`)
- Any changes to energy weights, sampling density, or the split/merge logic
