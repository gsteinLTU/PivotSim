import { describe, it, expect } from 'vitest';
import {
  euclideanDelta, angularDelta, segmentDuration, lerpPose,
  applyRotationPropagation, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED,
} from './utils.js';

describe('euclideanDelta', () => {
  it('computes 3D distance', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0 };
    const b = { x: 3, y: 4, z: 0, yaw: 0, pitch: 0, roll: 0 };
    expect(euclideanDelta(a, b)).toBeCloseTo(5, 5);
  });
});

describe('angularDelta', () => {
  it('sums absolute differences in yaw/pitch/roll', () => {
    const a = { x: 0, y: 0, z: 0, yaw: 0,   pitch: 0,   roll: 0   };
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

describe('applyRotationPropagation', () => {
  function makePoses(n) {
    return Array.from({ length: n }, (_, i) => ({
      x: i, y: 0, z: 0, yaw: 0, pitch: 0, roll: 0,
    }));
  }

  it('shifts yaw for indices in [startIdx, endIdx)', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 0.5);
    expect(result[0].yaw).toBeCloseTo(0);
    expect(result[1].yaw).toBeCloseTo(0);
    expect(result[2].yaw).toBeCloseTo(0.5);
    expect(result[3].yaw).toBeCloseTo(0.5);
    expect(result[4].yaw).toBeCloseTo(0);
  });

  it('leaves x/y/z untouched', () => {
    const poses  = makePoses(5);
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
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 2, 4, 'yaw', 1.0);
    expect(result[1].yaw).toBeCloseTo(0);
    expect(result[2].yaw).toBeCloseTo(1);
    expect(result[3].yaw).toBeCloseTo(1);
    expect(result[4].yaw).toBeCloseTo(0);
  });

  it('backward range: startIdx=1, endIdx=i+1 shifts all interior keypoints up to i', () => {
    const poses  = makePoses(5);
    const result = applyRotationPropagation(poses, 1, 3, 'pitch', 0.7);
    expect(result[0].pitch).toBeCloseTo(0);
    expect(result[1].pitch).toBeCloseTo(0.7);
    expect(result[2].pitch).toBeCloseTo(0.7);
    expect(result[3].pitch).toBeCloseTo(0);
  });
});
