import { checkCollisions } from './collision.js';
import { getEndpoints } from './path.js';

export const MAX_LINEAR_SPEED  = 0.5;   // m/s
export const MAX_ANGULAR_SPEED = 0.5;   // rad/s
const CLEARANCE_CAP = 0.3;              // m — clearance reward capped here
const DOFS = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'];
const SIGMA = { x: 0.1, y: 0.1, z: 0.1, yaw: 0.3, pitch: 0.2, roll: 0.2 };

export const DEFAULT_WEIGHTS = {
  w_col: 100, w_clr: 1, w_rot: 0.1, w_pos: 0.5, w_time: 0.01,
};

// ── Pure math helpers ──────────────────────────────────────────────────────

export function euclideanDelta(a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  return Math.sqrt(dx*dx + dy*dy + dz*dz);
}

export function angularDelta(a, b) {
  return Math.abs(b.yaw - a.yaw) + Math.abs(b.pitch - a.pitch) + Math.abs(b.roll - a.roll);
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
    yaw:   a.yaw   + (b.yaw   - a.yaw)   * t,
    pitch: a.pitch + (b.pitch - a.pitch) * t,
    roll:  a.roll  + (b.roll  - a.roll)  * t,
  };
}

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

// ── Segment energy ─────────────────────────────────────────────────────────

/**
 * Evaluates collision and clearance energy for one segment (a → b).
 * Returns { collEnergy, clrEnergy, duration }
 */
export function evalSegment(a, b, collisionQuads, halfExtents) {
  const dur = segmentDuration(a, b);
  const K = Math.max(5, Math.ceil(dur / 0.2));

  // First pass: K evenly-spaced samples
  let worstC = Infinity, worstT = 0.5;
  const clearances = [];
  for (let k = 0; k < K; k++) {
    const t = (k + 0.5) / K;
    const { minClearance } = checkCollisions(computeOBB(lerpPose(a, b, t), halfExtents), collisionQuads);
    clearances.push(minClearance);
    if (minClearance < worstC) { worstC = minClearance; worstT = t; }
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
  return { collEnergy, clrEnergy, duration: dur };
}

function totalEnergy(segData, poses, w) {
  let E = 0;
  for (let i = 0; i < segData.length; i++) {
    E += w.w_col  * segData[i].collEnergy;
    E -= w.w_clr  * segData[i].clrEnergy;
    E += w.w_rot  * angularDelta(poses[i], poses[i + 1]);
    E += w.w_pos  * euclideanDelta(poses[i], poses[i + 1]);
    E += w.w_time * segData[i].duration;
  }
  return E;
}

// ── Optimizer ──────────────────────────────────────────────────────────────

/**
 * Runs simulated annealing to find a collision-free trajectory.
 * @param {object[]} collisionQuads  from buildStairwell
 * @param {number[]} halfExtents     [hW, hH, hL] from getHalfExtents
 * @param {object}   centerline      from buildCenterline
 * @param {object}   weights         SA weights + optional maxIter override
 * @param {function} onProgress      called every 500 iters with current best
 * @param {function} shouldCancel    returns true to stop early
 * @returns {Promise<TrajectoryResult>}
 */
export async function optimizeTrajectory(
  collisionQuads, halfExtents, centerline, weights, onProgress, shouldCancel,
) {
  const w = { ...DEFAULT_WEIGHTS, ...(weights ?? {}) };
  const MAX_ITER = weights?.maxIter ?? 50000;
  const BATCH    = 500;

  const { start, end } = getEndpoints(centerline);
  const { points } = centerline;

  // Pull endpoints inward (toward the stairs) by half the box length + 5 cm padding
  // so the box starts fully inside the hallway rather than centered on the end wall.
  const ENDPOINT_PAD = 0.05;
  const inset = halfExtents[2] + ENDPOINT_PAD;
  function insetPoint(pt, toward) {
    const dx = toward[0] - pt[0], dy = toward[1] - pt[1], dz = toward[2] - pt[2];
    const len = Math.sqrt(dx*dx + dy*dy + dz*dz) || 1;
    return [pt[0] + dx/len * inset, pt[1] + dy/len * inset, pt[2] + dz/len * inset];
  }
  const startPt = insetPoint(start, points[1]);
  const endPt   = insetPoint(end,   points[points.length - 2]);

  // Endpoints fixed; y lifted by halfExtents[1] so box sits on floor
  const startPose = { x: startPt[0], y: startPt[1] + halfExtents[1], z: startPt[2], yaw: 0, pitch: 0, roll: 0 };
  const endPose   = { x: endPt[0],   y: endPt[1]   + halfExtents[1], z: endPt[2],   yaw: 0, pitch: 0, roll: 0 };

  let poses   = [startPose, endPose];
  let segData = [evalSegment(startPose, endPose, collisionQuads, halfExtents)];
  let energy  = totalEnergy(segData, poses, w);

  let bestPoses   = poses.map(p => ({ ...p }));
  let bestSegData = segData.map(s => ({ ...s }));
  let bestEnergy  = energy;

  const T_START = 5.0, T_END = 0.001;
  const COOL = Math.pow(T_END / T_START, 1 / Math.max(MAX_ITER, 1));
  let T = T_START;

  for (let iter = 0; iter < MAX_ITER; iter++) {
    if (shouldCancel?.()) break;
    T *= COOL;

    const r = Math.random();
    let newPoses, newSegData, newEnergy;

    if (r < 0.7 && poses.length > 2) {
      // ── Perturb ──────────────────────────────────────────────────────────
      const i   = 1 + Math.floor(Math.random() * (poses.length - 2));
      const dof = DOFS[Math.floor(Math.random() * 6)];
      newPoses   = poses.map(p => ({ ...p }));
      newPoses[i] = { ...poses[i], [dof]: poses[i][dof] + randn() * SIGMA[dof] * T };
      newSegData  = segData.slice();
      newSegData[i - 1] = evalSegment(newPoses[i - 1], newPoses[i],     collisionQuads, halfExtents);
      newSegData[i]     = evalSegment(newPoses[i],     newPoses[i + 1], collisionQuads, halfExtents);
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else if (r < 0.9 || poses.length <= 2) {
      // ── Split (also used as fallback when merge is impossible) ────────────
      // ── Split ─────────────────────────────────────────────────────────────
      let worstIdx = 0;
      for (let i = 1; i < segData.length; i++) {
        if (segData[i].collEnergy > segData[worstIdx].collEnergy) worstIdx = i;
      }
      const mid = lerpPose(poses[worstIdx], poses[worstIdx + 1], 0.5);
      for (const dof of DOFS) mid[dof] += randn() * SIGMA[dof] * T * 0.5;

      newPoses = [
        ...poses.slice(0, worstIdx + 1),
        mid,
        ...poses.slice(worstIdx + 1),
      ];
      newSegData = [
        ...segData.slice(0, worstIdx),
        evalSegment(newPoses[worstIdx],     mid,                    collisionQuads, halfExtents),
        evalSegment(mid,                    newPoses[worstIdx + 2], collisionQuads, halfExtents),
        ...segData.slice(worstIdx + 1),
      ];
      newEnergy = totalEnergy(newSegData, newPoses, w);

    } else {
      // ── Merge ─────────────────────────────────────────────────────────────
      const i = 1 + Math.floor(Math.random() * (poses.length - 2));
      newPoses = [...poses.slice(0, i), ...poses.slice(i + 1)];
      newSegData = [
        ...segData.slice(0, i - 1),
        evalSegment(newPoses[i - 1], newPoses[i], collisionQuads, halfExtents),
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
