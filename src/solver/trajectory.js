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

// Pure-math OBB — same convention as box.js computeOBB (YXZ Euler), no Three.js.
// R = Ry(yaw) × Rx(pitch) × Rz(roll); columns are local-axis world directions.
function computeOBB({ x, y, z, yaw, pitch, roll }, halfExtents) {
  const cy = Math.cos(yaw),   sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cr = Math.cos(roll),  sr = Math.sin(roll);
  return {
    center: [x, y, z],
    axes: [
      [ cy*cr + sy*sp*sr,  cp*sr, -sy*cr + cy*sp*sr ],  // local X
      [-cy*sr + sy*sp*cr,  cp*cr,  sy*sr + cy*sp*cr ],  // local Y
      [ sy*cp,            -sp,     cy*cp             ],  // local Z
    ],
    halfExtents,
  };
}

// Box-Muller normal sample
function randn() {
  const u = 1 - Math.random(), v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

// ── Containment check ──────────────────────────────────────────────────────

/**
 * Returns the "excess" outside the nearest OBB (positive = outside all OBBs).
 * Each OBB: { center: [x,y,z], axes: [[…],[…],[…]], halfExtents: [hx,hy,hz] }
 */
function signedDistFromOBBs(px, py, pz, obbs) {
  let best = Infinity;
  for (const obb of obbs) {
    const dx = px - obb.center[0], dy = py - obb.center[1], dz = pz - obb.center[2];
    let maxExcess = -Infinity;
    for (let i = 0; i < 3; i++) {
      const ax = obb.axes[i];
      const proj = Math.abs(dx * ax[0] + dy * ax[1] + dz * ax[2]);
      const excess = proj - obb.halfExtents[i];
      if (excess > maxExcess) maxExcess = excess;
    }
    if (maxExcess < best) best = maxExcess; // most-inside OBB
  }
  return best; // ≤ 0 = inside at least one OBB; > 0 = outside all, distance ≈ best
}

// ── Segment energy ─────────────────────────────────────────────────────────

/**
 * Evaluates collision, clearance, and void energy for one segment (a → b).
 * containmentOBBs: optional array of OBBs from buildContainmentOBBs; samples
 *   outside all OBBs accumulate voidEnergy (excess² per sample, time-proportional
 *   since K scales with duration).
 * Returns { collEnergy, clrEnergy, voidEnergy, duration }
 */
export function evalSegment(a, b, collisionQuads, halfExtents, containmentOBBs) {
  const dur  = segmentDuration(a, b);
  const dist = euclideanDelta(a, b);
  const K = Math.max(5, Math.ceil(dur / 0.2), Math.ceil(dist / 0.05));

  // First pass: K evenly-spaced samples
  let worstC = Infinity, worstT = 0.5;
  const clearances = [];
  let voidEnergy = 0;
  for (let k = 0; k < K; k++) {
    const t = (k + 0.5) / K;
    const pose = lerpPose(a, b, t);
    const { minClearance } = checkCollisions(computeOBB(pose, halfExtents), collisionQuads);
    clearances.push(minClearance);
    if (minClearance < worstC) { worstC = minClearance; worstT = t; }

    if (containmentOBBs?.length) {
      const excess = signedDistFromOBBs(pose.x, pose.y, pose.z, containmentOBBs);
      if (excess > 0) voidEnergy += excess * excess;
    }
  }

  // Second pass: 10 refined samples around the worst point (±halfSpacing in 5 steps)
  const halfSpacing = 0.5 / K;
  for (let r = 1; r <= 5; r++) {
    for (const s of [-1, 1]) {
      const t = Math.max(0.01, Math.min(0.99, worstT + s * (r / 5) * halfSpacing));
      const { minClearance } = checkCollisions(computeOBB(lerpPose(a, b, t), halfExtents), collisionQuads);
      clearances.push(minClearance);
    }
  }

  let collEnergy = 0, clrEnergy = 0;
  for (const c of clearances) {
    if (c < 0) collEnergy += c * c;  // (-c)² = c² since c < 0
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
  // Per-keypoint cost: each interior point must earn its place by reducing
  // collision/void energy by at least w_nk, preventing fidgety accumulation.
  E += w.w_nk * (poses.length - 2);
  return E;
}

// ── Optimizer ──────────────────────────────────────────────────────────────

/**
 * Runs simulated annealing to find a collision-free trajectory.
 * @param {object[]} collisionQuads   from buildStairwell
 * @param {number[]} halfExtents      [hW, hH, hL] from getHalfExtents
 * @param {object}   centerline       from buildCenterline
 * @param {object[]|null} containmentOBBs  from buildContainmentOBBs; null disables void penalty
 * @param {object}   weights          SA weights + optional maxIter override
 * @param {function} onProgress       called every 500 iters with current best
 * @param {function} shouldCancel     returns true to stop early
 * @returns {Promise<TrajectoryResult>}
 */
export async function optimizeTrajectory(
  collisionQuads, halfExtents, centerline, containmentOBBs, weights, onProgress, shouldCancel,
) {
  const w = { ...DEFAULT_WEIGHTS, ...(weights ?? {}) };
  const MAX_ITER = weights?.maxIter ?? 50000;
  const obbs = containmentOBBs ?? [];
  const BATCH    = 500;

  const { points } = centerline;

  // Place start/end at hallway midpoint so the box begins fully inside.
  function midpoint(a, b) {
    return [(a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2];
  }
  const startPt = midpoint(points[0], points[1]);
  const endPt   = midpoint(points[points.length - 1], points[points.length - 2]);

  // Endpoints centered vertically in the hallway (floor + ceiling midpoint)
  const halfCeil = centerline.ceilingHeight / 2;
  const startPose = { x: startPt[0], y: startPt[1] + halfCeil, z: startPt[2], yaw: 0, pitch: 0, roll: 0 };
  const endPose   = { x: endPt[0],   y: endPt[1]   + halfCeil, z: endPt[2],   yaw: 0, pitch: 0, roll: 0 };

  // Seed keypoints at stair junctions and flight midpoint so the path is
  // threaded through the stairwell from the start rather than needing SA to find it.
  const n = points.length;
  function poseAt(pt) {
    return { x: pt[0], y: pt[1] + halfCeil, z: pt[2], yaw: 0, pitch: 0, roll: 0 };
  }
  const stairBasePose = poseAt(points[1]);           // bottom stair junction
  const stairMidPt    = points[Math.floor(n / 2)];
  const midPose       = poseAt(stairMidPt);          // stair flight midpoint
  const stairTopPose  = poseAt(points[n - 2]);       // top stair junction

  // Bounding box of all collision quad vertices — perturbed poses are clamped
  // to this region so they can't escape to void space with zero collision energy.
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
    evalSegment(startPose,   stairBasePose, collisionQuads, halfExtents, obbs),
    evalSegment(stairBasePose, midPose,     collisionQuads, halfExtents, obbs),
    evalSegment(midPose,     stairTopPose,  collisionQuads, halfExtents, obbs),
    evalSegment(stairTopPose, endPose,      collisionQuads, halfExtents, obbs),
  ];
  let energy  = totalEnergy(segData, poses, w);

  let bestPoses   = poses.map(p => ({ ...p }));
  let bestSegData = segData.map(s => ({ ...s }));
  let bestEnergy  = energy;

  const T_START = 10.0, T_END = 0.001;
  const COOL = Math.pow(T_END / T_START, 1 / Math.max(MAX_ITER, 1));
  let T = T_START;

  for (let iter = 0; iter < MAX_ITER; iter++) {
    if (shouldCancel?.()) break;
    T *= COOL;

    const r = Math.random();
    let newPoses, newSegData, newEnergy;

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
      // All segments from [i-1] through [lastInterior] are affected by the rotation.
      const i    = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof  = ROTATION_DOFS[Math.floor(Math.random() * 3)];
      const delta = randn() * SIGMA[dof] * T;
      newPoses   = applyRotationPropagation(poses, i, poses.length - 1, dof, delta);
      newSegData  = segData.slice();
      const lastInterior = poses.length - 2;
      for (let j = i - 1; j <= lastInterior; j++) {
        newSegData[j] = evalSegment(newPoses[j], newPoses[j + 1], collisionQuads, halfExtents, obbs);
      }
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else if (r < 0.75 && poses.length > 2) {
      // ── Backward propagate ───────────────────────────────────────────────
      // Shift poses[1..i] by same delta on one rotation DOF.
      // All segments from [0] through [i] are affected by the rotation.
      const i    = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof  = ROTATION_DOFS[Math.floor(Math.random() * 3)];
      const delta = randn() * SIGMA[dof] * T;
      newPoses   = applyRotationPropagation(poses, 1, i + 1, dof, delta);
      newSegData  = segData.slice();
      for (let j = 0; j <= i; j++) {
        newSegData[j] = evalSegment(newPoses[j], newPoses[j + 1], collisionQuads, halfExtents, obbs);
      }
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

    // Metropolis acceptance
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
      const segmentTimes = bestSegData.map(s => s.duration);
      const totalTime    = segmentTimes.reduce((a, b) => a + b, 0);
      onProgress?.({ poses: bestPoses, segmentTimes, totalTime,
                     energy: bestEnergy, temperature: T, iteration: iter + 1 });
      await new Promise(resolve => setTimeout(resolve, 0));
    }
  }

  // Final result
  const segmentTimes = bestSegData.map(s => s.duration);
  const totalTime    = segmentTimes.reduce((a, b) => a + b, 0);
  const fits         = bestSegData.every(s => s.collEnergy === 0);

  let tightestIndex = 0, minC = Infinity;
  for (let i = 0; i < bestPoses.length; i++) {
    const { minClearance } = checkCollisions(computeOBB(bestPoses[i], halfExtents), collisionQuads);
    if (minClearance < minC) { minC = minClearance; tightestIndex = i; }
  }

  return { poses: bestPoses, segmentTimes, totalTime, fits, tightestIndex, finalEnergy: bestEnergy };
}
