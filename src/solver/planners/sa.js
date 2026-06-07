import { checkCollisions } from '../collision.js';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose, applyRotationPropagation,
  computeOBBFromPose,
} from '../utils.js';
import { findGatewayConfigs, bestGatewayConfig } from '../gateway.js';

export const DEFAULT_WEIGHTS = {
  w_col: 100, w_clr: 1.5, w_rot: 0.45, w_pos: 0.45, w_time: 0.5, w_void: 150, w_nk: 2,
};

const CLEARANCE_CAP = 0.3;
const DOFS          = ['x', 'y', 'z', 'yaw', 'pitch', 'roll'];
const ROTATION_DOFS = ['yaw', 'pitch', 'roll'];
const SIGMA         = { x: 0.1, y: 0.1, z: 0.1, yaw: 0.3, pitch: 0.2, roll: 0.2 };

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
    const { minClearance } = checkCollisions(computeOBBFromPose(pose, halfExtents), collisionQuads);
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
        computeOBBFromPose(lerpPose(a, b, t), halfExtents), collisionQuads,
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
    const {
      collisionQuads, halfExtents, startPose, endPose, containmentOBBs, centerline,
      quadsBySegment, boundaries, stairZone,
    } = context;
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
    let stairBasePose, stairTopPose;
    if (quadsBySegment && boundaries) {
      stairBasePose = bestGatewayConfig(findGatewayConfigs(
        boundaries.bottomTransitionPt, ceilingHeight,
        quadsBySegment['bottom-hall'], quadsBySegment.stair, halfExtents,
      )) ?? poseAt(points[Math.min(1, n - 1)]);

      stairTopPose = bestGatewayConfig(findGatewayConfigs(
        boundaries.topTransitionPt, ceilingHeight,
        quadsBySegment.stair, quadsBySegment['top-hall'], halfExtents,
      )) ?? poseAt(points[Math.max(n - 2, 0)]);
    } else {
      stairBasePose = poseAt(points[Math.min(1, n - 1)]);
      stairTopPose  = poseAt(points[Math.max(n - 2, 0)]);
    }
    const stairMidPt = points[Math.floor(n / 2)];
    const midPose    = poseAt(stairMidPt);

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
        const zone = stairZone ?? { zMin: -Infinity, zMax: Infinity };

        // Only insert waypoints within the stair zone; skip hallway segments
        let worstIdx = -1;
        for (let i = 0; i < segData.length; i++) {
          const midZ = (poses[i].z + poses[i + 1].z) / 2;
          if (midZ < zone.zMin || midZ > zone.zMax) continue;
          if (worstIdx === -1 ||
              segData[i].collEnergy > segData[worstIdx].collEnergy ||
             (segData[i].collEnergy === segData[worstIdx].collEnergy &&
              segData[i].duration > segData[worstIdx].duration)) {
            worstIdx = i;
          }
        }

        if (worstIdx === -1) {
          // No stair-zone segment is worst; fall back to single-DOF perturbation
          const i   = 1 + Math.floor(Math.random() * (poses.length - 2));
          const dof = DOFS[Math.floor(Math.random() * 6)];
          newPoses    = poses.map(p => ({ ...p }));
          newPoses[i] = clampPose({ ...poses[i], [dof]: poses[i][dof] + randn() * SIGMA[dof] * T });
          newSegData   = segData.slice();
          newSegData[i - 1] = evalSegment(newPoses[i - 1], newPoses[i],     collisionQuads, halfExtents, obbs);
          newSegData[i]     = evalSegment(newPoses[i],     newPoses[i + 1], collisionQuads, halfExtents, obbs);
          newEnergy = totalEnergy(newSegData, newPoses, w);
        } else {
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
        }

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
      const { minClearance } = checkCollisions(computeOBBFromPose(bestPoses[i], halfExtents), collisionQuads);
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
