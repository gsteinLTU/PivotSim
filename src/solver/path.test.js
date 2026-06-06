import { describe, it, expect } from 'vitest';
import { buildCenterline, getEndpoints } from './path.js';
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
    // btRad=90°: start should be near [hallwayLength, 0, 0]
    expect(start[0]).toBeCloseTo(DEFAULTS.hallwayLength, 3);
    expect(start[1]).toBeCloseTo(0, 3);
    expect(start[2]).toBeCloseTo(0, 3);
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
