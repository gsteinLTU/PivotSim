# Propagate-Rotation SA Moves Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add forward-propagate and backward-propagate move types to the simulated-annealing loop in `optimizeTrajectory` so the optimizer can build sustained-rotation plateaus across multiple keypoints in a single step.

**Architecture:** Extract a pure helper `applyRotationPropagation(poses, startIdx, endIdx, dof, delta)` that returns a new pose array with the given rotation DOF shifted for all keypoints in `[startIdx, endIdx)`. The SA loop calls this helper for forward (shift `i..n-2`) and backward (shift `1..i`) propagate moves, then re-evaluates only the two boundary segments that actually change. All keypoints remain plain `{x, y, z, yaw, pitch, roll}` objects — no new types.

**Tech Stack:** Vanilla JS, Vitest

---

## File Map

| File | Change |
|---|---|
| `src/solver/trajectory.js` | Export `applyRotationPropagation`; add `ROTATION_DOFS` const; add forward/backward propagate branches in SA loop; adjust probability thresholds |
| `src/solver/trajectory.test.js` | Add tests for `applyRotationPropagation` |

---

### Task 1: Export and test `applyRotationPropagation`

**Files:**
- Modify: `src/solver/trajectory.js`
- Test: `src/solver/trajectory.test.js`

- [ ] **Step 1: Write the failing tests**

Add to `src/solver/trajectory.test.js` after the existing `lerpPose` block:

```js
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose,
  evalSegment, optimizeTrajectory, applyRotationPropagation,
  MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
} from './trajectory.js';
```

(Update the existing import at the top of the file — add `applyRotationPropagation` to the list.)

Then add these tests after the `lerpPose` describe block:

```js
describe('applyRotationPropagation', () => {
  function makePoses(n) {
    return Array.from({ length: n }, (_, i) => ({
      x: i, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0,
    }));
  }

  it('shifts yaw for indices in [startIdx, endIdx)', () => {
    const poses = makePoses(5); // indices 0..4
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 0.5);
    expect(result[0].yaw).toBeCloseTo(0);
    expect(result[1].yaw).toBeCloseTo(0);
    expect(result[2].yaw).toBeCloseTo(0.5);
    expect(result[3].yaw).toBeCloseTo(0.5);
    expect(result[4].yaw).toBeCloseTo(0);
  });

  it('leaves x/y/z untouched', () => {
    const poses = makePoses(5);
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
    const poses = makePoses(5); // n=5, interior = 1,2,3
    // forward from i=2: shift poses[2] and poses[3], leave poses[0],poses[1],poses[4]
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 1.0);
    expect(result[1].yaw).toBeCloseTo(0); // not shifted
    expect(result[2].yaw).toBeCloseTo(1); // shifted
    expect(result[3].yaw).toBeCloseTo(1); // shifted
    expect(result[4].yaw).toBeCloseTo(0); // end endpoint — not shifted
  });

  it('backward range: startIdx=1, endIdx=i+1 shifts all interior keypoints up to i', () => {
    const poses = makePoses(5);
    // backward to i=2: shift poses[1] and poses[2], leave poses[0],poses[3],poses[4]
    const result = applyRotationPropagation(poses, 1, 3, 'pitch', 0.7);
    expect(result[0].pitch).toBeCloseTo(0); // start endpoint — not shifted
    expect(result[1].pitch).toBeCloseTo(0.7); // shifted
    expect(result[2].pitch).toBeCloseTo(0.7); // shifted
    expect(result[3].pitch).toBeCloseTo(0);   // not shifted
  });
});
```

- [ ] **Step 2: Run the tests to confirm they fail**

```bash
npx vitest run src/solver/trajectory.test.js
```

Expected: tests fail because `applyRotationPropagation` is not exported.

- [ ] **Step 3: Implement `applyRotationPropagation` in `trajectory.js`**

Add this after the `lerpPose` function (around line 51) in `src/solver/trajectory.js`:

```js
/**
 * Returns a new poses array where poses[startIdx..endIdx-1][dof] += delta.
 * Used by forward and backward propagate SA moves.
 * Only rotation DOFs (yaw, pitch, roll) should be passed as dof.
 */
export function applyRotationPropagation(poses, startIdx, endIdx, dof, delta) {
  return poses.map((p, j) =>
    j >= startIdx && j < endIdx ? { ...p, [dof]: p[dof] + delta } : { ...p }
  );
}
```

- [ ] **Step 4: Run the tests to confirm they pass**

```bash
npx vitest run src/solver/trajectory.test.js
```

Expected: all new tests pass.

- [ ] **Step 5: Commit**

```bash
git add src/solver/trajectory.js src/solver/trajectory.test.js
git commit -m "feat: export applyRotationPropagation helper with tests"
```

---

### Task 2: Integrate forward/backward propagate moves into the SA loop

**Files:**
- Modify: `src/solver/trajectory.js`

- [ ] **Step 1: Add `ROTATION_DOFS` constant**

At the top of `src/solver/trajectory.js`, after the existing `const DOFS` line (line 7), add:

```js
const ROTATION_DOFS = ['yaw', 'pitch', 'roll'];
```

- [ ] **Step 2: Replace the SA move selection block**

Find the current if/else block in the SA loop (currently around line 255). It currently reads:

```js
    if (r < 0.7 && poses.length > 2) {
      // ── Perturb ──────────────────────────────────────────────────────────
```

Replace the entire if/else chain (through the closing `}` of the merge branch, around line 304) with:

```js
    if (r < 0.55 && poses.length > 2) {
      // ── Point perturb ────────────────────────────────────────────────────
      const i   = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof = DOFS[Math.floor(Math.random() * 6)];
      newPoses   = poses.map(p => ({ ...p }));
      newPoses[i] = clampPose({ ...poses[i], [dof]: poses[i][dof] + randn() * SIGMA[dof] * T });
      newSegData  = segData.slice();
      newSegData[i - 1] = evalSegment(newPoses[i - 1], newPoses[i],     collisionQuads, halfExtents, obbs);
      newSegData[i]     = evalSegment(newPoses[i],     newPoses[i + 1], collisionQuads, halfExtents, obbs);
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else if (r < 0.65 && poses.length > 2) {
      // ── Forward propagate ────────────────────────────────────────────────
      // Shift poses[i..n-2] by same delta on one rotation DOF.
      // Only two boundary segments change: [i-1] and [n-2].
      const i    = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof  = ROTATION_DOFS[Math.floor(Math.random() * 3)];
      const delta = randn() * SIGMA[dof] * T;
      newPoses   = applyRotationPropagation(poses, i, poses.length - 1, dof, delta);
      newSegData  = segData.slice();
      const lastInterior = poses.length - 2;
      newSegData[i - 1]     = evalSegment(newPoses[i - 1], newPoses[i],                   collisionQuads, halfExtents, obbs);
      newSegData[lastInterior] = evalSegment(newPoses[lastInterior], newPoses[lastInterior + 1], collisionQuads, halfExtents, obbs);
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else if (r < 0.75 && poses.length > 2) {
      // ── Backward propagate ───────────────────────────────────────────────
      // Shift poses[1..i] by same delta on one rotation DOF.
      // Only two boundary segments change: [0] and [i].
      const i    = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof  = ROTATION_DOFS[Math.floor(Math.random() * 3)];
      const delta = randn() * SIGMA[dof] * T;
      newPoses   = applyRotationPropagation(poses, 1, i + 1, dof, delta);
      newSegData  = segData.slice();
      newSegData[0] = evalSegment(newPoses[0], newPoses[1], collisionQuads, halfExtents, obbs);
      newSegData[i] = evalSegment(newPoses[i], newPoses[i + 1], collisionQuads, halfExtents, obbs);
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else if (r < 0.90 || poses.length <= 2) {
      // ── Split ─────────────────────────────────────────────────────────────
      // Primary: highest collision energy. Tiebreak: longest segment.
      // Without the tiebreak, all splits target index 0 when nothing is colliding,
      // piling keypoints in the first (hallway) segment.
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
      // ── Merge ─────────────────────────────────────────────────────────────
      const i = 1 + Math.floor(Math.random() * (poses.length - 2));
      newPoses = [...poses.slice(0, i), ...poses.slice(i + 1)];
      newSegData = [
        ...segData.slice(0, i - 1),
        evalSegment(newPoses[i - 1], newPoses[i], collisionQuads, halfExtents, obbs),
        ...segData.slice(i + 1),
      ];
      newEnergy = totalEnergy(newSegData, newPoses, w);
    }
```

- [ ] **Step 3: Handle the edge case where i === lastInterior in forward propagate**

When `i === poses.length - 2` (only one keypoint is shifted), `newSegData[i - 1]` and `newSegData[lastInterior]` refer to adjacent segments. This is correct as-is — both get re-evaluated independently. No additional handling needed, but verify by tracing through with `poses.length = 3` (i=1, lastInterior=1): `newSegData[0]` and `newSegData[1]` both get re-evaluated, which is correct since only poses[1] changed.

- [ ] **Step 4: Run the full test suite**

```bash
npm test
```

Expected: all existing tests pass, no regressions.

- [ ] **Step 5: Commit**

```bash
git add src/solver/trajectory.js
git commit -m "feat: add forward/backward propagate SA moves for sustained rotation"
```
