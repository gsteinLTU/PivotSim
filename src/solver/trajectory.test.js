import { describe, it, expect } from 'vitest';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose,
  evalSegment, optimizeTrajectory,
  MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
} from './trajectory.js';

// ── Pure math helpers ──────────────────────────────────────────────────────

describe('euclideanDelta', () => {
  it('computes 3D distance', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 3, y: 4, z: 0, yaw: 0, pitch: 0, roll: 0 };
    expect(euclideanDelta(a, b)).toBeCloseTo(5, 5);
  });
});

describe('angularDelta', () => {
  it('sums absolute differences in yaw/pitch/roll', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0,   pitch: 0,  roll: 0   };
    const b = { x: 0, y: 0, z: 0, yaw: 0.5, pitch: 0.3, roll: 0.1 };
    expect(angularDelta(a, b)).toBeCloseTo(0.9, 5);
  });
});

describe('segmentDuration', () => {
  it('uses linear speed for position-dominant segments', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 1, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    expect(segmentDuration(a, b)).toBeCloseTo(1 / MAX_LINEAR_SPEED, 5);
  });

  it('uses angular speed for rotation-dominant segments', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0,                pitch: 0, roll: 0 };
    const b = { x: 0, y: 0, z: 0, yaw: MAX_ANGULAR_SPEED, pitch: 0, roll: 0 };
    // angular = MAX_ANGULAR_SPEED rad, duration = 1s exactly
    expect(segmentDuration(a, b)).toBeCloseTo(1.0, 5);
  });
});

describe('lerpPose', () => {
  const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
  const b = { x: 2, y: 4, z: 6, yaw: 1, pitch: 0.5, roll: -0.5 };

  it('returns a at t=0', () => {
    const r = lerpPose(a, b, 0);
    expect(r.x).toBeCloseTo(0); expect(r.yaw).toBeCloseTo(0);
  });

  it('returns b at t=1', () => {
    const r = lerpPose(a, b, 1);
    expect(r.x).toBeCloseTo(2); expect(r.yaw).toBeCloseTo(1);
  });

  it('returns midpoint at t=0.5', () => {
    const r = lerpPose(a, b, 0.5);
    expect(r.x).toBeCloseTo(1); expect(r.z).toBeCloseTo(3); expect(r.yaw).toBeCloseTo(0.5);
  });
});

// ── evalSegment ────────────────────────────────────────────────────────────

// Wide open space: floor at y=0, ceiling at y=10, nothing else
const openQuads = [
  { type: 'floor',   vertices: [[-20,0,-20],[20,0,-20],[20,0,20],[-20,0,20]],   normal: [0,1,0] },
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
    // Wall at x=0.1, box half-width=0.5 — clips if box center is at x=0
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

// ── optimizeTrajectory ─────────────────────────────────────────────────────

const openCenterline = {
  points: [[0, 0, -5], [0, 0, 5]],
  totalLength: 10,
  ceilingHeight: 2.4,
};

describe('optimizeTrajectory', () => {
  it('returns a TrajectoryResult with correct shape', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline,
      null, { maxIter: 100 }, null, null
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
      openQuads, tinyHalf, openCenterline,
      null, { maxIter: 50 }, null, null
    );
    const sum = result.segmentTimes.reduce((a, b) => a + b, 0);
    expect(result.totalTime).toBeCloseTo(sum, 5);
  });

  it('fits === true immediately for an open space with a tiny box', async () => {
    const result = await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline,
      null, { maxIter: 10 }, null, null
    );
    // Start/end are at y=0.05 (half-height above floor), path is in open space → fits immediately
    expect(result.fits).toBe(true);
  });

  it('calls onProgress with the right shape', async () => {
    const calls = [];
    await optimizeTrajectory(
      openQuads, tinyHalf, openCenterline,
      null, { maxIter: 600 },
      (p) => calls.push(p),
      null
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
      openQuads, tinyHalf, openCenterline,
      null, { maxIter: 50000 },
      () => { callCount++; },
      () => callCount >= 1   // cancel after first progress callback
    );
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    expect(callCount).toBeLessThan(10);  // canceled early
  });
});
