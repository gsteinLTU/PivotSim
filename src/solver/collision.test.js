import { describe, it, expect } from 'vitest';
import { testOBBvsQuad, checkCollisions } from './collision.js';

const SQRT_HALF = Math.SQRT1_2; // ≈ 0.7071

// ─── Shared fixtures ────────────────────────────────────────────────────────

// Floor at y=0, 2m × 2m centered at origin
const floorQuad = {
  type: 'floor',
  vertices: [[-1, 0, -1], [1, 0, -1], [1, 0, 1], [-1, 0, 1]],
  normal: [0, 1, 0],
};

// Ceiling at y=2.4
const ceilQuad = {
  type: 'ceiling',
  vertices: [[-1, 2.4, -1], [1, 2.4, -1], [1, 2.4, 1], [-1, 2.4, 1]],
  normal: [0, -1, 0],
};

// Vertical wall at x=1, spanning y 0–2, z –2 to 2
const wallQuad = {
  type: 'wall-left',
  vertices: [[1, 0, -2], [1, 0, 2], [1, 2, 2], [1, 2, -2]],
  normal: [-1, 0, 0],
};

// Identity OBB well above the floor
const clearOBB = {
  center: [0, 2, 0],
  axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  halfExtents: [0.25, 0.25, 0.25],
};

// Identity OBB centred on the floor plane → collision
const collidingOBB = {
  center: [0, 0, 0],
  axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  halfExtents: [0.25, 0.25, 0.25],
};

// OBB with 45° yaw, long in its local Z (world [+X,0,+X] direction)
const rotatedOBB_far = {
  center: [-1.0, 1.0, 0.0],
  axes: [[SQRT_HALF, 0, -SQRT_HALF], [0, 1, 0], [SQRT_HALF, 0, SQRT_HALF]],
  halfExtents: [0.3, 0.5, 2.0],
};

const rotatedOBB_close = {
  center: [0.0, 1.0, 0.0],
  axes: [[SQRT_HALF, 0, -SQRT_HALF], [0, 1, 0], [SQRT_HALF, 0, SQRT_HALF]],
  halfExtents: [0.3, 0.5, 2.0],
};

// ─── testOBBvsQuad ──────────────────────────────────────────────────────────

describe('testOBBvsQuad', () => {
  it('reports no collision for OBB above floor', () => {
    const result = testOBBvsQuad(clearOBB, floorQuad);
    expect(result.collides).toBe(false);
    expect(result.clearance).toBeCloseTo(1.75, 4);
  });

  it('reports collision for OBB penetrating floor', () => {
    const result = testOBBvsQuad(collidingOBB, floorQuad);
    expect(result.collides).toBe(true);
  });

  it('reports no collision for rotated OBB past the wall', () => {
    // 45°-yaw OBB centred at x=-1, extends ~1.626 in world +X → max reach x≈0.626 < 1
    const result = testOBBvsQuad(rotatedOBB_far, wallQuad);
    expect(result.collides).toBe(false);
    expect(result.clearance).toBeCloseTo(0.374, 2);
  });

  it('reports collision for rotated OBB overlapping the wall', () => {
    // Same OBB centred at x=0 → max reach x≈1.626 > 1
    const result = testOBBvsQuad(rotatedOBB_close, wallQuad);
    expect(result.collides).toBe(true);
  });
});

// ─── checkCollisions ────────────────────────────────────────────────────────

describe('checkCollisions', () => {
  it('returns no collision when OBB clears all quads', () => {
    const result = checkCollisions(clearOBB, [floorQuad, ceilQuad]);
    expect(result.collides).toBe(false);
    expect(result.contactQuads).toHaveLength(0);
  });

  it('minClearance is the closest gap across all quads', () => {
    // clearOBB centre at y=2.0, halfExtent 0.25
    // → gap to floor (y=0) on normal [0,1,0]: 1.75
    // → gap to ceil (y=2.4) on normal [0,-1,0]: 0.15
    const result = checkCollisions(clearOBB, [floorQuad, ceilQuad]);
    expect(result.minClearance).toBeCloseTo(0.15, 2);
  });

  it('reports collision and identifies which quads are hit', () => {
    const farCeil = {
      type: 'ceiling',
      vertices: [[-1, 5, -1], [1, 5, -1], [1, 5, 1], [-1, 5, 1]],
      normal: [0, -1, 0],
    };
    const result = checkCollisions(collidingOBB, [floorQuad, farCeil]);
    expect(result.collides).toBe(true);
    expect(result.contactQuads).toContain(floorQuad);
    expect(result.contactQuads).not.toContain(farCeil);
  });

  it('returns minClearance 0 when colliding', () => {
    const result = checkCollisions(collidingOBB, [floorQuad]);
    expect(result.collides).toBe(true);
    expect(result.minClearance).toBe(0);
  });
});
