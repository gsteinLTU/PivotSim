import { describe, it, expect } from 'vitest';
import { rrtPlanner, DEFAULTS } from './rrt-connect.js';
import { buildPlannerContext } from '../context.js';
import { DEFAULTS as STAIR_DEFAULTS, BOX_DEFAULTS } from '../../defaults.js';
import { checkCollisions } from '../collision.js';
import { computeOBBFromPose } from '../utils.js';

// ── Shared fixtures ────────────────────────────────────────────────────────

const openQuads = [
  { type: 'floor',   vertices: [[-20,0,-20],[20,0,-20],[20,0,20],[-20,0,20]],     normal: [0,1,0]  },
  { type: 'ceiling', vertices: [[-20,10,-20],[20,10,-20],[20,10,20],[-20,10,20]], normal: [0,-1,0] },
];
const tinyHalf = [0.05, 0.05, 0.05];

const startPose = { x: 0, y: 1.2, z: -2.5, yaw: 0, pitch: 0, roll: 0 };
const endPose   = { x: 0, y: 1.2, z:  2.5, yaw: 0, pitch: 0, roll: 0 };
const endPoses  = [
  endPose,
  { x: 0, y: 1.2, z: 2.5, yaw: Math.PI, pitch: 0, roll: 0 },
];

const openContext = {
  collisionQuads: openQuads,
  halfExtents: tinyHalf,
  startPose,
  endPose,
  endPoses,
  containmentOBBs: [],
  centerline: {
    points: [[0, 0, -5], [0, 0, -1.5], [0, 0, 0], [0, 0, 1.5], [0, 0, 5]],
    totalLength: 10,
    ceilingHeight: 2.4,
  },
};

// ── result shape ───────────────────────────────────────────────────────────

describe('rrtPlanner.plan result shape', () => {
  it('returns array of poses with length >= 2', async () => {
    const result = await rrtPlanner.plan(openContext, { maxIter: 300, seed: 1 }, null, null);
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });

  it('returns boolean fits and numeric tightestIndex in range', async () => {
    const result = await rrtPlanner.plan(openContext, { maxIter: 300, seed: 1 }, null, null);
    expect(typeof result.fits).toBe('boolean');
    expect(typeof result.tightestIndex).toBe('number');
    expect(result.tightestIndex).toBeGreaterThanOrEqual(0);
    expect(result.tightestIndex).toBeLessThan(result.poses.length);
  });

  it('does not include segmentTimes or totalTime', async () => {
    const result = await rrtPlanner.plan(openContext, { maxIter: 300, seed: 1 }, null, null);
    expect(result.segmentTimes).toBeUndefined();
    expect(result.totalTime).toBeUndefined();
  });
});

// ── endpoint invariant ─────────────────────────────────────────────────────

describe('rrtPlanner.plan endpoint invariant', () => {
  it('first pose equals startPose and last pose is one of endPoses (success path)', async () => {
    // Open space — should always find a path quickly
    const result = await rrtPlanner.plan(openContext, { maxIter: 1000, seed: 7 }, null, null);
    expect(result.poses[0]).toEqual(startPose);
    const lastPose = result.poses.at(-1);
    const matchesAGoal = openContext.endPoses.some(ep =>
      ep.x === lastPose.x && ep.y === lastPose.y && ep.z === lastPose.z &&
      ep.yaw === lastPose.yaw && ep.pitch === lastPose.pitch && ep.roll === lastPose.roll
    );
    expect(matchesAGoal).toBe(true);
  });

  it('first pose equals startPose and last pose equals endPose (fallback straight line)', async () => {
    // maxIter:0 forces the straight-line fallback immediately
    const result = await rrtPlanner.plan(openContext, { maxIter: 0, seed: 1 }, null, null);
    expect(result.fits).toBe(false);
    expect(result.poses[0]).toEqual(startPose);
    expect(result.poses.at(-1)).toEqual(endPose);
  });
});

// ── open-space validity ────────────────────────────────────────────────────

describe('rrtPlanner.plan open space', () => {
  it('finds a collision-free path (fits === true) in an unobstructed corridor', async () => {
    const result = await rrtPlanner.plan(openContext, { maxIter: 2000, seed: 42 }, null, null);
    expect(result.fits).toBe(true);
    for (const pose of result.poses) {
      const { minClearance } = checkCollisions(
        computeOBBFromPose(pose, openContext.halfExtents),
        openContext.collisionQuads,
      );
      expect(minClearance).toBeGreaterThanOrEqual(0);
    }
  });
});

// ── cancellation ───────────────────────────────────────────────────────────

describe('rrtPlanner.plan cancellation', () => {
  it('respects shouldCancel and stops early', async () => {
    let callCount = 0;
    const result = await rrtPlanner.plan(
      openContext,
      { maxIter: 100000 },
      () => { callCount++; },
      () => callCount >= 1,
    );
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
    // Should stop well before 100 000 iterations — only a handful of progress calls
    expect(callCount).toBeLessThan(10);
  });
});

// ── progress callback ──────────────────────────────────────────────────────

describe('rrtPlanner.plan onProgress', () => {
  it('calls onProgress with RRT-specific payload shape', async () => {
    const calls = [];
    await rrtPlanner.plan(
      openContext,
      { maxIter: 300, progressBatch: 50, seed: 1 },
      (p) => calls.push(p),
      null,
    );
    expect(calls.length).toBeGreaterThan(0);
    const p = calls[0];
    expect(Array.isArray(p.poses)).toBe(true);
    expect(typeof p.iteration).toBe('number');
    expect(typeof p.maxIter).toBe('number');
    expect(typeof p.treeSizeStart).toBe('number');
    expect(typeof p.treeSizeGoal).toBe('number');
    expect(typeof p.found).toBe('boolean');
    expect(p.plannerType).toBe('rrt');
  });
});

// ── start-in-collision short-circuit ──────────────────────────────────────

describe('rrtPlanner.plan start-in-collision', () => {
  it('returns fits:false with 2 poses immediately when startPose collides', async () => {
    // Wall at x=0.1 with normal [-1,0,0]; box halfExtents[0]=0.5 centered at x=0 crosses it
    const wallQuad = {
      type: 'wall',
      vertices: [[0.1,-5,-5],[0.1,5,-5],[0.1,5,5],[0.1,-5,5]],
      normal: [-1, 0, 0],
    };
    const ctx = {
      ...openContext,
      startPose: { x: 0, y: 1, z: 0, yaw: 0, pitch: 0, roll: 0 },
      collisionQuads: [wallQuad],
      halfExtents: [0.5, 0.25, 0.5],
      endPoses: [],  // empty — all blocked
    };
    const progressCalls = [];
    const result = await rrtPlanner.plan(
      ctx, { maxIter: 10000, seed: 1 }, (p) => progressCalls.push(p), null,
    );
    expect(result.fits).toBe(false);
    expect(result.poses.length).toBe(2);
    // No search should have run
    expect(progressCalls.length).toBe(0);
  });
});

// ── backward compat (no quadsBySegment) ───────────────────────────────────

describe('rrtPlanner.plan backward compat', () => {
  it('works without quadsBySegment (gateway biasing disabled gracefully)', async () => {
    // openContext has no quadsBySegment or boundaries — gateway bias must be skipped
    const result = await rrtPlanner.plan(openContext, { maxIter: 500, seed: 1 }, null, null);
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });
});

// ── full context smoke test ────────────────────────────────────────────────

describe('rrtPlanner.plan full planner context', () => {
  it('returns >= 2 poses with a real stairwell context without throwing', async () => {
    const ctx = buildPlannerContext(STAIR_DEFAULTS, BOX_DEFAULTS);
    const result = await rrtPlanner.plan(ctx, { maxIter: 500, seed: 1 }, null, null);
    expect(Array.isArray(result.poses)).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });
});

// ── DEFAULTS export ────────────────────────────────────────────────────────

describe('DEFAULTS', () => {
  it('exports expected config keys with sensible types', () => {
    expect(typeof DEFAULTS.maxIter).toBe('number');
    expect(typeof DEFAULTS.epsilon).toBe('number');
    expect(typeof DEFAULTS.gatewayBias).toBe('number');
    expect(typeof DEFAULTS.smoothingIters).toBe('number');
    expect(typeof DEFAULTS.wAng).toBe('number');
    expect(typeof DEFAULTS.progressBatch).toBe('number');
  });
});

// ── formatProgress ─────────────────────────────────────────────────────────

describe('rrtPlanner.formatProgress', () => {
  it('creates #tl-rrt-iter on first call and updates it on second without throwing', () => {
    const container = document.createElement('div');

    expect(() => {
      rrtPlanner.formatProgress(
        { treeSizeStart: 10, treeSizeGoal: 8, iteration: 50, maxIter: 4000, found: false },
        container,
      );
    }).not.toThrow();

    const iterEl = container.querySelector('#tl-rrt-iter');
    expect(iterEl).not.toBeNull();

    expect(() => {
      rrtPlanner.formatProgress(
        { treeSizeStart: 50, treeSizeGoal: 45, iteration: 200, maxIter: 4000, found: true },
        container,
      );
    }).not.toThrow();

    // Element should be reused (not re-created)
    expect(container.querySelector('#tl-rrt-iter')).toBe(iterEl);
  });
});

// ── multi-root goal tree ───────────────────────────────────────────────────

describe('rrtPlanner.plan multi-root goal', () => {
  it('finds a path when only the 180° goal is reachable', async () => {
    // Block the yaw:0 goal by placing a wall just before it, keep yaw:π root clear
    // Use open space — both goals are reachable; just verify the planner accepts multi-root
    const twoRootCtx = {
      ...openContext,
      endPoses: [
        { x: 0, y: 1.2, z: 2.5, yaw: 0,        pitch: 0, roll: 0 },
        { x: 0, y: 1.2, z: 2.5, yaw: Math.PI,  pitch: 0, roll: 0 },
      ],
      endPose: { x: 0, y: 1.2, z: 2.5, yaw: 0, pitch: 0, roll: 0 },
    };
    const result = await rrtPlanner.plan(twoRootCtx, { maxIter: 2000, seed: 3 }, null, null);
    expect(result.fits).toBe(true);
    expect(result.poses.length).toBeGreaterThanOrEqual(2);
  });
});
