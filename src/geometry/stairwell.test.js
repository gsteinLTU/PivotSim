import { describe, it, expect } from 'vitest';
import { buildStairwell } from './stairwell.js';
import { DEFAULTS } from '../defaults.js';

describe('buildStairwell', () => {
  it('returns a group and collisionQuads', () => {
    const result = buildStairwell(DEFAULTS);
    expect(result).toHaveProperty('group');
    expect(result).toHaveProperty('collisionQuads');
    expect(result.group.isGroup).toBe(true);
    expect(Array.isArray(result.collisionQuads)).toBe(true);
  });

  it('group contains children for the stair flight', () => {
    const result = buildStairwell(DEFAULTS);
    // At minimum: step treads + step risers
    expect(result.group.children.length).toBeGreaterThan(0);
  });

  it('total rise equals numSteps * risePerStep', () => {
    const params = { ...DEFAULTS, numSteps: 10, risePerStep: 0.2 };
    const result = buildStairwell(params);
    // The highest collision quad y-coordinate should be at total rise
    const totalRise = params.numSteps * params.risePerStep;
    const maxY = Math.max(
      ...result.collisionQuads
        .filter((q) => q.type === 'tread')
        .flatMap((q) => q.vertices.map((v) => v[1]))
    );
    expect(maxY).toBeCloseTo(totalRise, 5);
  });

  it('total run equals numSteps * runPerStep', () => {
    const params = { ...DEFAULTS, numSteps: 10, runPerStep: 0.3 };
    const result = buildStairwell(params);
    const totalRun = params.numSteps * params.runPerStep;
    const maxZ = Math.max(
      ...result.collisionQuads
        .filter((q) => q.type === 'tread')
        .flatMap((q) => q.vertices.map((v) => v[2]))
    );
    expect(maxZ).toBeCloseTo(totalRun, 5);
  });

  it('generates one tread quad per step', () => {
    const params = { ...DEFAULTS, numSteps: 8 };
    const result = buildStairwell(params);
    const treads = result.collisionQuads.filter((q) => q.type === 'tread');
    expect(treads.length).toBe(8);
  });

  it('generates one riser quad per step', () => {
    const params = { ...DEFAULTS, numSteps: 8 };
    const result = buildStairwell(params);
    const risers = result.collisionQuads.filter((q) => q.type === 'riser');
    expect(risers.length).toBe(8);
  });

  it('tread quads have correct width', () => {
    const params = { ...DEFAULTS, stairWidth: 1.5 };
    const result = buildStairwell(params);
    const tread = result.collisionQuads.find((q) => q.type === 'tread');
    // Tread spans x-axis from -width/2 to +width/2
    const xs = tread.vertices.map((v) => v[0]);
    const treadWidth = Math.max(...xs) - Math.min(...xs);
    expect(treadWidth).toBeCloseTo(1.5, 5);
  });

  it('omits ceiling quad when slopedCeiling is false', () => {
    const params = { ...DEFAULTS, slopedCeiling: false };
    const result = buildStairwell(params);
    const stairCeilings = result.collisionQuads.filter(
      (q) => q.type === 'ceiling'
    );
    // Only hallway ceilings, no stair ceiling
    const hallwayCeilings = stairCeilings.length;
    const paramsWithCeiling = { ...DEFAULTS, slopedCeiling: true };
    const resultWith = buildStairwell(paramsWithCeiling);
    const allCeilings = resultWith.collisionQuads.filter((q) => q.type === 'ceiling').length;
    expect(hallwayCeilings).toBeLessThan(allCeilings);
  });

  it('hallway collision quads are correctly rotated for 90° turn', () => {
    const params = { ...DEFAULTS, bottomHallwayTurn: 90 };
    const result = buildStairwell(params);
    const floors = result.collisionQuads.filter((q) => q.type === 'floor');
    // At least one floor quad should have vertices extending in the +X direction
    const hasXExtent = floors.some((q) =>
      q.vertices.some((v) => v[0] > 1.0)
    );
    expect(hasXExtent).toBe(true);
  });

  // ─── Wall quads ──────────────────────────────────────────────────────────

  it('generates wall-left and wall-right quads for each step', () => {
    const params = { ...DEFAULTS, numSteps: 6 };
    const result = buildStairwell(params);
    const leftWalls = result.collisionQuads.filter((q) => q.type === 'wall-left');
    const rightWalls = result.collisionQuads.filter((q) => q.type === 'wall-right');
    // Stair walls: one per step. Hallway walls: one per hallway (bottom + top).
    expect(leftWalls.length).toBe(6 + 2);
    expect(rightWalls.length).toBe(6 + 2);
  });

  it('stair wall quads are at ±stairWidth/2 in X', () => {
    const params = { ...DEFAULTS, stairWidth: 1.2 };
    const result = buildStairwell(params);
    const halfW = 1.2 / 2;
    // Filter to stair walls (exclude hallway walls by checking normal direction)
    const leftWalls = result.collisionQuads.filter(
      (q) => q.type === 'wall-left' && q.normal[0] === 1
    );
    const rightWalls = result.collisionQuads.filter(
      (q) => q.type === 'wall-right' && q.normal[0] === -1
    );
    // All stair left wall vertices should be at x = -halfW
    for (const wall of leftWalls) {
      for (const v of wall.vertices) {
        expect(v[0]).toBeCloseTo(-halfW, 4);
      }
    }
    for (const wall of rightWalls) {
      for (const v of wall.vertices) {
        expect(v[0]).toBeCloseTo(halfW, 4);
      }
    }
  });

  // ─── Collision quad normals ──────────────────────────────────────────────

  it('tread normals point upward', () => {
    const result = buildStairwell(DEFAULTS);
    const treads = result.collisionQuads.filter((q) => q.type === 'tread');
    for (const t of treads) {
      expect(t.normal).toEqual([0, 1, 0]);
    }
  });

  it('riser normals point toward the stair base (-Z)', () => {
    const result = buildStairwell(DEFAULTS);
    const risers = result.collisionQuads.filter((q) => q.type === 'riser');
    for (const r of risers) {
      expect(r.normal).toEqual([0, 0, -1]);
    }
  });

  // ─── Top hallway turn ───────────────────────────────────────────────────

  it('top hallway rotated 90° extends in -X at top of stairs', () => {
    const params = { ...DEFAULTS, topHallwayTurn: 90 };
    const result = buildStairwell(params);
    const totalRise = params.numSteps * params.risePerStep;
    // Find floor quads near the top (y ≈ totalRise)
    const topFloors = result.collisionQuads.filter(
      (q) => q.type === 'floor' && q.vertices.some((v) => Math.abs(v[1] - totalRise) < 0.01)
    );
    // At least one floor should extend in the -X direction
    const hasNegX = topFloors.some((q) =>
      q.vertices.some((v) => v[0] < -1.0)
    );
    expect(hasNegX).toBe(true);
  });

  // ─── Negative turn angle ────────────────────────────────────────────────

  it('negative bottom hallway turn extends in -X direction', () => {
    const params = { ...DEFAULTS, bottomHallwayTurn: -90 };
    const result = buildStairwell(params);
    const floors = result.collisionQuads.filter((q) => q.type === 'floor');
    const hasNegX = floors.some((q) =>
      q.vertices.some((v) => v[0] < -1.0)
    );
    expect(hasNegX).toBe(true);
  });

  // ─── End cap quads ──────────────────────────────────────────────────────

  it('generates wall-end quads when hallway is turned', () => {
    const params = { ...DEFAULTS, bottomHallwayTurn: 90 };
    const result = buildStairwell(params);
    const endCaps = result.collisionQuads.filter((q) => q.type === 'wall-end');
    expect(endCaps.length).toBeGreaterThanOrEqual(1);
  });

  it('generates exactly 2 wall-end quads when hallway is straight (one per hallway)', () => {
    const params = { ...DEFAULTS, bottomHallwayTurn: 0, topHallwayTurn: 0 };
    const result = buildStairwell(params);
    const endCaps = result.collisionQuads.filter((q) => q.type === 'wall-end');
    expect(endCaps.length).toBe(2);
  });
});
