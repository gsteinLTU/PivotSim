import { describe, it, expect } from 'vitest';
import * as THREE from 'three';
import { buildQuadDebug } from './debug.js';

const TREAD_QUAD = {
  type: 'tread',
  vertices: [
    [-0.5, 0.2, 0],
    [0.5, 0.2, 0],
    [0.5, 0.2, 0.28],
    [-0.5, 0.2, 0.28],
  ],
  normal: [0, 1, 0],
};

describe('buildQuadDebug', () => {
  it('returns a THREE.Group', () => {
    const group = buildQuadDebug([TREAD_QUAD]);
    expect(group).toBeInstanceOf(THREE.Group);
  });

  it('returns empty group for empty input', () => {
    const group = buildQuadDebug([]);
    expect(group.children).toHaveLength(0);
  });

  it('adds 2 children per quad (LineLoop + ArrowHelper)', () => {
    const riserQuad = { ...TREAD_QUAD, type: 'riser', normal: [0, 0, -1] };
    const group = buildQuadDebug([TREAD_QUAD, riserQuad]);
    expect(group.children).toHaveLength(4);
  });

  it('first child per quad is a LineLoop', () => {
    const group = buildQuadDebug([TREAD_QUAD]);
    expect(group.children[0]).toBeInstanceOf(THREE.LineLoop);
  });

  it('second child per quad is an ArrowHelper', () => {
    const group = buildQuadDebug([TREAD_QUAD]);
    expect(group.children[1]).toBeInstanceOf(THREE.ArrowHelper);
  });

  it('uses correct color for known type (tread = 0x00ff44)', () => {
    const group = buildQuadDebug([TREAD_QUAD]);
    const line = group.children[0];
    expect(line.material.color.getHex()).toBe(0x00ff44);
  });

  it('uses gray (0x888888) for unknown types', () => {
    const group = buildQuadDebug([{ ...TREAD_QUAD, type: 'unknown-type' }]);
    const line = group.children[0];
    expect(line.material.color.getHex()).toBe(0x888888);
  });

  it('LineLoop has 4 vertex positions', () => {
    const group = buildQuadDebug([TREAD_QUAD]);
    const line = group.children[0];
    const posAttr = line.geometry.getAttribute('position');
    expect(posAttr.count).toBe(4);
  });
});
