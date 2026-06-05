import { describe, it, expect } from 'vitest';
import { computeOBB, getOBBCorners, getHalfExtents } from './box.js';

describe('getHalfExtents', () => {
  it('returns [width/2, height/2, length/2]', () => {
    expect(getHalfExtents({ length: 2.0, width: 0.8, height: 0.5 }))
      .toEqual([0.4, 0.25, 1.0]);
  });
});

describe('computeOBB', () => {
  it('at zero rotation produces identity axes', () => {
    const obb = computeOBB(
      { x: 1, y: 2, z: 3, yaw: 0, pitch: 0, roll: 0 },
      [0.4, 0.25, 1.0]
    );
    expect(obb.center).toEqual([1, 2, 3]);
    expect(obb.halfExtents).toEqual([0.4, 0.25, 1.0]);
    expect(obb.axes[0][0]).toBeCloseTo(1);
    expect(obb.axes[0][1]).toBeCloseTo(0);
    expect(obb.axes[0][2]).toBeCloseTo(0);
    expect(obb.axes[1][0]).toBeCloseTo(0);
    expect(obb.axes[1][1]).toBeCloseTo(1);
    expect(obb.axes[1][2]).toBeCloseTo(0);
    expect(obb.axes[2][0]).toBeCloseTo(0);
    expect(obb.axes[2][1]).toBeCloseTo(0);
    expect(obb.axes[2][2]).toBeCloseTo(1);
  });

  it('90° yaw rotates local X to world -Z and local Z to world +X', () => {
    const obb = computeOBB(
      { x: 0, y: 0, z: 0, yaw: Math.PI / 2, pitch: 0, roll: 0 },
      [0.4, 0.25, 1.0]
    );
    // local X (width axis) → world -Z
    expect(obb.axes[0][0]).toBeCloseTo(0);
    expect(obb.axes[0][1]).toBeCloseTo(0);
    expect(obb.axes[0][2]).toBeCloseTo(-1);
    // local Z (length axis) → world +X
    expect(obb.axes[2][0]).toBeCloseTo(1);
    expect(obb.axes[2][1]).toBeCloseTo(0);
    expect(obb.axes[2][2]).toBeCloseTo(0);
  });
});

describe('getOBBCorners', () => {
  it('returns 8 corners', () => {
    const obb = {
      center: [0, 0, 0],
      axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      halfExtents: [1, 2, 3],
    };
    expect(getOBBCorners(obb).length).toBe(8);
  });

  it('identity OBB corners span ±halfExtents on each axis', () => {
    const obb = {
      center: [0, 0, 0],
      axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
      halfExtents: [1, 2, 3],
    };
    const corners = getOBBCorners(obb);
    const xs = corners.map((c) => c[0]);
    const ys = corners.map((c) => c[1]);
    const zs = corners.map((c) => c[2]);
    expect(Math.min(...xs)).toBeCloseTo(-1);
    expect(Math.max(...xs)).toBeCloseTo(1);
    expect(Math.min(...ys)).toBeCloseTo(-2);
    expect(Math.max(...ys)).toBeCloseTo(2);
    expect(Math.min(...zs)).toBeCloseTo(-3);
    expect(Math.max(...zs)).toBeCloseTo(3);
  });

  it('rotated OBB corners are in world space', () => {
    // 90° yaw: local Z (halfExtent=3) now points in world +X
    const obb = {
      center: [0, 0, 0],
      axes: [[0, 0, -1], [0, 1, 0], [1, 0, 0]], // 90° yaw
      halfExtents: [1, 2, 3],
    };
    const corners = getOBBCorners(obb);
    const xs = corners.map((c) => c[0]);
    // local Z (halfExtent=3) is now world +X, local X (halfExtent=1) is world -Z
    // World X span: ±3 (from length axis)
    expect(Math.min(...xs)).toBeCloseTo(-3);
    expect(Math.max(...xs)).toBeCloseTo(3);
  });
});
