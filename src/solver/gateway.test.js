import { describe, it, expect } from 'vitest';
import { findGatewayConfigs, bestGatewayConfig } from './gateway.js';

const wideOpenQuads = [
  { type: 'floor',   vertices: [[-10,0,-10],[10,0,-10],[10,0,10],[-10,0,10]],    normal: [0,1,0]  },
  { type: 'ceiling', vertices: [[-10,3,-10],[10,3,-10],[10,3,10],[-10,3,10]],    normal: [0,-1,0] },
  { type: 'wall-left',  vertices: [[-2,0,-10],[-2,0,10],[-2,3,10],[-2,3,-10]],  normal: [1,0,0]  },
  { type: 'wall-right', vertices: [[ 2,0,-10],[ 2,0,10],[ 2,3,10],[ 2,3,-10]], normal: [-1,0,0] },
];
const smallHalf = [0.1, 0.15, 0.2];

describe('findGatewayConfigs', () => {
  it('finds valid configs in wide open space', () => {
    const configs = findGatewayConfigs([0, 0, 0], 3.0, wideOpenQuads, [], smallHalf);
    expect(configs.length).toBeGreaterThan(0);
  });

  it('returns empty array when box cannot fit regardless of orientation', () => {
    const tinyGap = [
      { type: 'floor',   vertices: [[-10,0,-10],[10,0,-10],[10,0,10],[-10,0,10]],      normal: [0,1,0]  },
      { type: 'ceiling', vertices: [[-10,0.05,-10],[10,0.05,-10],[10,0.05,10],[-10,0.05,10]], normal: [0,-1,0] },
    ];
    const bigBox = [1.0, 1.0, 1.0];
    const configs = findGatewayConfigs([0, 0, 0], 0.05, tinyGap, [], bigBox);
    expect(configs.length).toBe(0);
  });

  it('every returned config is a valid 6-DOF pose object', () => {
    const configs = findGatewayConfigs([0, 0, 0], 3.0, wideOpenQuads, [], smallHalf);
    for (const c of configs) {
      for (const k of ['x', 'y', 'z', 'yaw', 'pitch', 'roll']) {
        expect(typeof c[k], `key ${k}`).toBe('number');
        expect(Number.isFinite(c[k]), `key ${k} is finite`).toBe(true);
      }
    }
  });

  it('configs have x and z matching the transition point', () => {
    const configs = findGatewayConfigs([1.5, 0, 2.3], 3.0, wideOpenQuads, [], smallHalf);
    for (const c of configs) {
      expect(c.x).toBeCloseTo(1.5, 5);
      expect(c.z).toBeCloseTo(2.3, 5);
    }
  });

  it('considers quads from both adjacent segments', () => {
    // quadsB has walls that restrict orientations further
    const blocker = [
      { type: 'wall-left', vertices: [[-0.15,0,-10],[-0.15,0,10],[-0.15,3,10],[-0.15,3,-10]], normal: [1,0,0] },
      { type: 'wall-right', vertices: [[0.15,0,-10],[0.15,0,10],[0.15,3,10],[0.15,3,-10]], normal: [-1,0,0] },
    ];
    const open  = findGatewayConfigs([0,0,0], 3.0, wideOpenQuads, [], smallHalf);
    const tight = findGatewayConfigs([0,0,0], 3.0, wideOpenQuads, blocker, smallHalf);
    expect(tight.length).toBeLessThan(open.length);
  });
});

describe('bestGatewayConfig', () => {
  it('returns null for empty array', () => {
    expect(bestGatewayConfig([])).toBeNull();
  });

  it('returns the config with smallest total angular magnitude', () => {
    const configs = [
      { x: 0, y: 1, z: 0, yaw: 1.5,  pitch: 0.8,  roll: 0.5  },
      { x: 0, y: 1, z: 0, yaw: 0.1,  pitch: 0.1,  roll: 0.05 },
      { x: 0, y: 1, z: 0, yaw: 2.0,  pitch: 1.0,  roll: 0.0  },
    ];
    const best = bestGatewayConfig(configs);
    expect(best.yaw).toBeCloseTo(0.1, 5);
    expect(best.pitch).toBeCloseTo(0.1, 5);
  });

  it('returns the single config when array has one element', () => {
    const c = { x: 0, y: 1, z: 0, yaw: 0.5, pitch: 0.3, roll: 0.1 };
    expect(bestGatewayConfig([c])).toBe(c);
  });
});
