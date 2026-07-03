/**
 * Integration tests for the RRT-Connect planner against real-world scenarios.
 *
 * Fast diagnostic tests run as part of the normal suite.
 * Full planner runs are in describe.skip blocks — to run them explicitly:
 *   npx vitest run src/solver/planners/rrt-connect.integration.test.js
 * then temporarily remove the `.skip` on the block you want.
 */

import { describe, it, expect } from 'vitest';
import { rrtPlanner } from './rrt-connect.js';
import { buildPlannerContext } from '../context.js';
import { DEFAULTS, BOX_DEFAULTS } from '../../defaults.js';
import { checkCollisions } from '../collision.js';
import { computeOBBFromPose } from '../utils.js';

// ── Shared scenario params ─────────────────────────────────────────────────

// 20" × 45" × 70" box (width × length × height)
const LARGE_BOX = { width: 0.508, length: 1.143, height: 1.778 };

// 84" tall × 40" wide hallways, 90° / -90° turns (all other params default)
const NARROW_STAIRWELL = {
  ...DEFAULTS,
  bottomHallwayWidth: 1.016,  // 40"
  topHallwayWidth:    1.016,  // 40"
  ceilingHeight:      2.134,  // 84"
  bottomHallwayTurn:  90,
  topHallwayTurn:    -90,
};

// ── Fast diagnostics (always run) ─────────────────────────────────────────
// These pin-point exactly which early-exit guard is firing without running
// the full planner.

describe('context diagnostics: default box + default stairwell', () => {
  it('startPose is collision-free', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    const { minClearance } = checkCollisions(
      computeOBBFromPose(ctx.startPose, ctx.halfExtents),
      ctx.collisionQuads,
    );
    expect(minClearance).toBeGreaterThanOrEqual(0);
  });

  it('endPoses has at least one valid entry', () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    expect(ctx.endPoses.length).toBeGreaterThan(0);
  });
});

describe('context diagnostics: large box + narrow stairwell', () => {
  it('startPose is collision-free', () => {
    const ctx = buildPlannerContext(NARROW_STAIRWELL, LARGE_BOX);
    const { minClearance } = checkCollisions(
      computeOBBFromPose(ctx.startPose, ctx.halfExtents),
      ctx.collisionQuads,
    );
    expect(minClearance).toBeGreaterThanOrEqual(0);
  });

  it('endPoses has at least one valid entry', () => {
    const ctx = buildPlannerContext(NARROW_STAIRWELL, LARGE_BOX);
    expect(ctx.endPoses.length).toBeGreaterThan(0);
  });
});

// ── Full planner runs (skipped by default — expensive) ────────────────────

describe.skip('planner integration: default box + default stairwell', () => {
  it('finds a collision-free path and calls onProgress at least once', { timeout: 30000 }, async () => {
    const ctx = buildPlannerContext(DEFAULTS, BOX_DEFAULTS);
    let progressCount = 0;
    const result = await rrtPlanner.plan(
      ctx,
      { maxIter: 10000, seed: 42 },
      () => { progressCount++; },
      null,
    );
    expect(progressCount).toBeGreaterThan(0);   // planner actually ran
    expect(result.fits).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });
});

describe.skip('planner integration: large box + narrow stairwell', () => {
  it('runs to iteration limit (does not give up instantly)', { timeout: 30000 }, async () => {
    const ctx = buildPlannerContext(NARROW_STAIRWELL, LARGE_BOX);
    let progressCount = 0;
    const result = await rrtPlanner.plan(
      ctx,
      { maxIter: 5000, seed: 1 },
      () => { progressCount++; },
      null,
    );
    // The planner may or may not find a path — but it MUST have run at least
    // one progress batch (not exited via the early-out guard).
    expect(progressCount).toBeGreaterThan(0);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });
});
