import { segmentDuration } from './utils.js';

export function buildTrajectory(poses) {
  const segmentTimes = poses.slice(0, -1).map((p, i) => segmentDuration(p, poses[i + 1]));
  const totalTime    = segmentTimes.reduce((a, b) => a + b, 0);
  return { poses, segmentTimes, totalTime };
}
