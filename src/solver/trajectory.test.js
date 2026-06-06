import { describe, it, expect } from 'vitest';
import {
  evalSegment, optimizeTrajectory,
} from './trajectory.js';

const openQuads = [
  { type: 'floor',   vertices: [[-20,0,-20],[20,0,-20],[20,0,20],[-20,0,20]],     normal: [0,1,0]  },
  { type: 'ceiling', vertices: [[-20,10,-20],[20,10,-20],[20,10,20],[-20,10,20]], normal: [0,-1,0] },
];
const tinyHalf = [0.05, 0.05, 0.05];

describe('evalSegment', () => {
  it('returns zero collEnergy for a clear segment far from surfaces', () => {
    const a = { x: 0, y: 5, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 5, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, openQuads, tinyHalf);
    expect(seg.collEnergy).toBe(0);
    expect(seg.clrEnergy).toBeGreaterThan(0);
    expect(seg.duration).toBeGreaterThan(0);
  });

  it('returns positive collEnergy when segment clips a wall', () => {
    const wallQuad = {
      type: 'wall',
      vertices: [[0.1,-5,-5],[0.1,5,-5],[0.1,5,5],[0.1,-5,5]],
      normal: [-1, 0, 0],
    };
    const a = { x: 0, y: 1, z: -1, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 0, y: 1, z:  1, yaw: 0, pitch: 0, roll: 0 };
    const seg = evalSegment(a, b, [wallQuad], [0.5, 0.25, 0.5]);
    expect(seg.collEnergy).toBeGreaterThan(0);
  });
});

const openCenterline = {
  points: [[0, 0, -5], [0, 0, 5]],
  totalLength: 10,
  ceilingHeight: 2.4,
};

describe('optimizeTrajectory', () => {
  it('returns a TrajectoryResult with correct shape', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 100 }, null, null
    );
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(Array.isArray(result.segmentTimes)).toBe(true);
    expect(result.segmentTimes.length).toBe(result.poses.length - 1);
    expect(typeof result.fits).toBe('boolean');
    expect(typeof result.tightestIndex).toBe('number');
    expect(result.tightestIndex).toBeGreaterThanOrEqual(0);
    expect(result.tightestIndex).toBeLessThan(result.poses.length);
  });

  it('totalTime equals sum of segmentTimes', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 50 }, null, null
    );
    const sum = result.segmentTimes.reduce((a, b) => a + b, 0);
    expect(result.totalTime).toBeCloseTo(sum, 5);
  });

  it('fits === true immediately for an open space with a tiny box', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 10 }, null, null
    );
    expect(result.fits).toBe(true);
  });

  it('calls onProgress with the right shape', async () => {
    const calls = [];
    await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 600 },
      (p) => calls.push(p), null
    );
    expect(calls.length).toBeGreaterThan(0);
    const p = calls[0];
    expect(Array.isArray(p.poses)).toBe(true);
    expect(typeof p.energy).toBe('number');
    expect(typeof p.temperature).toBe('number');
    expect(typeof p.iteration).toBe('number');
  });

  it('respects shouldCancel', async () => {
    let callCount = 0;
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline, null, { maxIter: 50000 },
      () => { callCount++; },
      () => callCount >= 1
    );
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(callCount).toBeLessThan(10);
  });
});
