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
});
