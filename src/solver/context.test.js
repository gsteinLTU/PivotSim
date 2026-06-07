import { describe, it, expect } from 'vitest';
import { buildPlannerContext } from './context.js';
import { DEFAULTS, BOX_DEFAULTS } from '../defaults.js';

describe('buildPlannerContext', () => {
  it('returns all required fields', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(Array.isArray(ctx.collisionQuads)).toBe(true);
    expect(ctx.collisionQuads.length).toBeGreaterThan(0);
    expect(Array.isArray(ctx.halfExtents)).toBe(true);
    expect(ctx.halfExtents).toHaveLength(3);
    expect(Array.isArray(ctx.containmentOBBs)).toBe(true);
    expect(ctx.centerline).toBeDefined();
    expect(ctx.startPose).toBeDefined();
    expect(ctx.endPose).toBeDefined();
  });

  it('startPose and endPose are valid 6-DOF poses', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    for (const pose of [ctx.startPose, ctx.endPose]) {
      for (const k of ['x', 'y', 'z', 'yaw', 'pitch', 'roll']) {
        expect(typeof pose[k]).toBe('number');
        expect(Number.isFinite(pose[k])).toBe(true);
      }
    }
  });

  it('endPose is above startPose (stairs go up)', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(ctx.endPose.y).toBeGreaterThan(ctx.startPose.y);
  });

  it('startPose and endPose are distinct', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    const dist = Math.sqrt(
      (ctx.endPose.x - ctx.startPose.x) ** 2 +
      (ctx.endPose.y - ctx.startPose.y) ** 2 +
      (ctx.endPose.z - ctx.startPose.z) ** 2
    );
    expect(dist).toBeGreaterThan(0.5);
  });

  it('halfExtents has positive values matching box dims', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    for (const h of ctx.halfExtents) {
      expect(h).toBeGreaterThan(0);
    }
    expect(ctx.halfExtents[0]).toBeCloseTo(BOX_DEFAULTS.width  / 2, 5);
    expect(ctx.halfExtents[1]).toBeCloseTo(BOX_DEFAULTS.height / 2, 5);
    expect(ctx.halfExtents[2]).toBeCloseTo(BOX_DEFAULTS.length / 2, 5);
  });

  it('returns quadsBySegment with stair, bottom-hall, top-hall keys', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(ctx.quadsBySegment).toBeDefined();
    expect(Array.isArray(ctx.quadsBySegment.stair)).toBe(true);
    expect(Array.isArray(ctx.quadsBySegment['bottom-hall'])).toBe(true);
    expect(Array.isArray(ctx.quadsBySegment['top-hall'])).toBe(true);
    expect(ctx.quadsBySegment.stair.length).toBeGreaterThan(0);
    expect(ctx.quadsBySegment['bottom-hall'].length).toBeGreaterThan(0);
    expect(ctx.quadsBySegment['top-hall'].length).toBeGreaterThan(0);
  });

  it('quadsBySegment partitions all collisionQuads with no overlap', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    const total = ctx.quadsBySegment.stair.length
      + ctx.quadsBySegment['bottom-hall'].length
      + ctx.quadsBySegment['top-hall'].length;
    expect(total).toBe(ctx.collisionQuads.length);
  });

  it('returns boundaries with transition points', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(Array.isArray(ctx.boundaries.bottomTransitionPt)).toBe(true);
    expect(Array.isArray(ctx.boundaries.topTransitionPt)).toBe(true);
  });
});
