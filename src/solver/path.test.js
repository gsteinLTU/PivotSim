import { describe, it, expect } from 'vitest';
import { buildCenterline, getEndpoints, buildContainmentOBBs, getSegmentBoundaries, getCorridorYaws } from './path.js';
import { DEFAULTS } from '../defaults.js';

describe('buildCenterline', () => {
  it('returns points array and totalLength', () => {
    const cl = buildCenterline(DEFAULTS);
    expect(Array.isArray(cl.points)).toBe(true);
    expect(cl.totalLength).toBeGreaterThan(0);
  });

  it('has at least numSteps + 4 points (junctions + hallway ends + steps)', () => {
    const cl = buildCenterline(DEFAULTS);
    expect(cl.points.length).toBeGreaterThanOrEqual(DEFAULTS.numSteps + 4);
  });

  it('first point is in the bottom hallway (near hallway end)', () => {
    const cl = buildCenterline(DEFAULTS);
    const start = cl.points[0];
    // btRad=90°, with stairHalfW/btHalfW offsets:
    //   x = L - stairHalfW, z = -btHalfW
    expect(start[0]).toBeCloseTo(DEFAULTS.hallwayLength - DEFAULTS.stairWidth / 2, 3);
    expect(start[1]).toBeCloseTo(0, 3);
    expect(start[2]).toBeCloseTo(-DEFAULTS.bottomHallwayWidth / 2, 3);
  });

  it('last point is in the top hallway', () => {
    const cl = buildCenterline(DEFAULTS);
    const end = cl.points[cl.points.length - 1];
    const totalRise = DEFAULTS.numSteps * DEFAULTS.risePerStep;
    expect(end[1]).toBeCloseTo(totalRise, 3);
  });

  it('totalLength matches sum of segment distances', () => {
    const cl = buildCenterline(DEFAULTS);
    let sum = 0;
    for (let i = 1; i < cl.points.length; i++) {
      const [x0, y0, z0] = cl.points[i - 1];
      const [x1, y1, z1] = cl.points[i];
      sum += Math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2);
    }
    expect(cl.totalLength).toBeCloseTo(sum, 5);
  });
});

describe('getEndpoints', () => {
  it('returns start and end as [x,y,z] arrays', () => {
    const cl = buildCenterline(DEFAULTS);
    const { start, end } = getEndpoints(cl);
    expect(start).toHaveLength(3);
    expect(end).toHaveLength(3);
  });

  it('start equals first point, end equals last point', () => {
    const cl = buildCenterline(DEFAULTS);
    const { start, end } = getEndpoints(cl);
    expect(start).toEqual(cl.points[0]);
    expect(end).toEqual(cl.points[cl.points.length - 1]);
  });
});

describe('buildContainmentOBBs', () => {
  it('returns 3 OBBs with center, axes, halfExtents', () => {
    const cl = buildCenterline(DEFAULTS);
    const obbs = buildContainmentOBBs(cl, DEFAULTS);
    expect(obbs).toHaveLength(3);
    for (const obb of obbs) {
      expect(obb).toHaveProperty('center');
      expect(obb).toHaveProperty('axes');
      expect(obb).toHaveProperty('halfExtents');
      expect(obb.center).toHaveLength(3);
      expect(obb.axes).toHaveLength(3);
      expect(obb.halfExtents).toHaveLength(3);
      expect(obb.halfExtents.every(h => h > 0)).toBe(true);
    }
  });

  it('start pose center is inside bottom hallway OBB', () => {
    const cl = buildCenterline(DEFAULTS);
    const obbs = buildContainmentOBBs(cl, DEFAULTS);
    // startPt = midpoint(points[0], points[1]), y = ceilingHeight/2
    const pts = cl.points;
    const sx = (pts[0][0] + pts[1][0]) / 2;
    const sy = DEFAULTS.ceilingHeight / 2;
    const sz = (pts[0][2] + pts[1][2]) / 2;
    const inside = obbs.some(obb => {
      const dx = sx - obb.center[0], dy = sy - obb.center[1], dz = sz - obb.center[2];
      return obb.axes.every((ax, i) =>
        Math.abs(dx * ax[0] + dy * ax[1] + dz * ax[2]) <= obb.halfExtents[i]
      );
    });
    expect(inside).toBe(true);
  });

  it('clearly void point is outside all OBBs', () => {
    const cl = buildCenterline(DEFAULTS);
    const obbs = buildContainmentOBBs(cl, DEFAULTS);
    // Far outside in X — no part of the stairwell extends to x=20
    const [px, py, pz] = [20, 1.2, 0];
    const inside = obbs.some(obb => {
      const dx = px - obb.center[0], dy = py - obb.center[1], dz = pz - obb.center[2];
      return obb.axes.every((ax, i) =>
        Math.abs(dx * ax[0] + dy * ax[1] + dz * ax[2]) <= obb.halfExtents[i]
      );
    });
    expect(inside).toBe(false);
  });
});

describe('getSegmentBoundaries', () => {
  it('returns correct indices for standard params', () => {
    const cl = buildCenterline(DEFAULTS);
    const b = getSegmentBoundaries(cl);
    expect(b.bottomTransitionIdx).toBe(1);
    expect(b.topTransitionIdx).toBe(cl.points.length - 2);
  });

  it('bottomTransitionPt is at the stair base [0, 0, 0]', () => {
    const cl = buildCenterline(DEFAULTS);
    const { bottomTransitionPt } = getSegmentBoundaries(cl);
    expect(bottomTransitionPt[0]).toBeCloseTo(0, 5);
    expect(bottomTransitionPt[1]).toBeCloseTo(0, 5);
    expect(bottomTransitionPt[2]).toBeCloseTo(0, 5);
  });

  it('topTransitionPt matches total rise and run', () => {
    const cl = buildCenterline(DEFAULTS);
    const { topTransitionPt } = getSegmentBoundaries(cl);
    const totalRise = DEFAULTS.numSteps * DEFAULTS.risePerStep;
    const totalRun  = DEFAULTS.numSteps * DEFAULTS.runPerStep;
    expect(topTransitionPt[1]).toBeCloseTo(totalRise, 4);
    expect(topTransitionPt[2]).toBeCloseTo(totalRun, 4);
  });
});

describe('getCorridorYaws', () => {
  it('returns finite numbers for startYaw and endYaw', () => {
    const cl = buildCenterline(DEFAULTS);
    const { startYaw, endYaw } = getCorridorYaws(cl);
    expect(Number.isFinite(startYaw)).toBe(true);
    expect(Number.isFinite(endYaw)).toBe(true);
  });

  it('returns 0 for both yaws on a straight stairwell (no turns)', () => {
    const straight = { ...DEFAULTS, bottomHallwayTurn: 0, topHallwayTurn: 0 };
    const cl = buildCenterline(straight);
    const { startYaw, endYaw } = getCorridorYaws(cl);
    expect(startYaw).toBeCloseTo(0, 5);
    expect(endYaw).toBeCloseTo(0, 5);
  });

  it('startYaw and endYaw are in [-π, π]', () => {
    const cl = buildCenterline(DEFAULTS);
    const { startYaw, endYaw } = getCorridorYaws(cl);
    expect(startYaw).toBeGreaterThanOrEqual(-Math.PI);
    expect(startYaw).toBeLessThanOrEqual(Math.PI);
    expect(endYaw).toBeGreaterThanOrEqual(-Math.PI);
    expect(endYaw).toBeLessThanOrEqual(Math.PI);
  });
});
