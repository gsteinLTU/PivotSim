import { describe, it, expect } from 'vitest';
import { buildTrajectory } from './trajectory.js';

describe('buildTrajectory', () => {
  it('returns segmentTimes with length poses.length - 1', () => {
    const poses = [
      { x: 0, y: 1, z: -3, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  0, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  3, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes } = buildTrajectory(poses);
    expect(segmentTimes).toHaveLength(2);
  });

  it('totalTime equals sum of segmentTimes', () => {
    const poses = [
      { x: 0, y: 1, z: -3, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  0, yaw: 0, pitch: 0, roll: 0 },
      { x: 0, y: 1, z:  3, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes, totalTime } = buildTrajectory(poses);
    expect(totalTime).toBeCloseTo(segmentTimes.reduce((a, b) => a + b, 0), 10);
  });

  it('each segment duration is positive', () => {
    const poses = [
      { x: 0, y: 1, z: 0, yaw: 0, pitch: 0, roll: 0 },
      { x: 1, y: 1, z: 0, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { segmentTimes } = buildTrajectory(poses);
    expect(segmentTimes[0]).toBeGreaterThan(0);
  });

  it('returns the same poses reference', () => {
    const poses = [
      { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 },
      { x: 1, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 },
    ];
    const { poses: result } = buildTrajectory(poses);
    expect(result).toBe(poses);
  });
});
