import { checkCollisions } from './collision.js';
import { computeOBBFromPose } from './utils.js';

const YAW_STEPS   = 12;  // 30° increments, full rotation
const PITCH_STEPS = 9;   // ~20° increments, −80° to +80°
const ROLL_STEPS  = 7;   // ~30° increments, −90° to +90°

/**
 * Returns all (position, orientation) poses at `transitionPt` that are
 * collision-free against the combined quads of both adjacent segments.
 *
 * Position X/Z is fixed to the transition point; Y is set to the vertical
 * midpoint of the corridor at that junction. Orientation is swept over a
 * discrete grid of yaw × pitch × roll.
 */
export function findGatewayConfigs(transitionPt, ceilingHeight, quadsA, quadsB, halfExtents) {
  const [tx, ty, tz] = transitionPt;
  const allQuads = [...quadsA, ...quadsB];
  if (allQuads.length === 0) return [];
  const y = ty + ceilingHeight / 2;
  const valid = [];

  for (let ai = 0; ai < YAW_STEPS; ai++) {
    const yaw = (ai / YAW_STEPS) * Math.PI * 2;
    for (let pi = 0; pi < PITCH_STEPS; pi++) {
      const pitch = -Math.PI * 0.44 + pi * (Math.PI * 0.88 / (PITCH_STEPS - 1));
      for (let ri = 0; ri < ROLL_STEPS; ri++) {
        const roll = -Math.PI * 0.5 + ri * (Math.PI / (ROLL_STEPS - 1));
        const pose = { x: tx, y, z: tz, yaw, pitch, roll };
        const { minClearance } = checkCollisions(computeOBBFromPose(pose, halfExtents), allQuads);
        if (minClearance >= 0) valid.push(pose);
      }
    }
  }

  return valid;
}

/**
 * Picks the gateway config most likely to lead to a successful plan.
 * Primary criterion: most level (min pitch² + roll²) — horizontal hallways don't need tilt.
 * Secondary criterion: min yaw² (wrapped to [-π, π]) — prefer corridor-aligned over arbitrary spin.
 * Returns null if configs is empty.
 */
export function bestGatewayConfig(configs) {
  if (configs.length === 0) return null;
  return configs.reduce((best, c) => {
    const yw = c.yaw > Math.PI ? c.yaw - 2 * Math.PI : c.yaw;
    const byw = best.yaw > Math.PI ? best.yaw - 2 * Math.PI : best.yaw;
    const score  = c.pitch * c.pitch    + c.roll * c.roll    + 0.1 * yw  * yw;
    const bscore = best.pitch * best.pitch + best.roll * best.roll + 0.1 * byw * byw;
    return score < bscore ? c : best;
  });
}
