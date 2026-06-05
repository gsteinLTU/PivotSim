import { describe, it, expect, vi } from 'vitest';

vi.mock('three', async (importOriginal) => {
  const THREE = await importOriginal();
  class MockWebGLRenderer {
    constructor() {
      this.setSize = vi.fn();
      this.setPixelRatio = vi.fn();
      this.domElement = document.createElement('canvas');
      this.render = vi.fn();
    }
  }
  return {
    ...THREE,
    WebGLRenderer: MockWebGLRenderer,
  };
});

vi.mock('three/addons/controls/OrbitControls.js', () => {
  class MockOrbitControls {
    constructor() {
      this.target = { set: vi.fn() };
      this.update = vi.fn();
    }
  }
  return { OrbitControls: MockOrbitControls };
});

import { createScene } from './scene.js';

describe('createScene', () => {
  it('returns scene, camera, renderer, and controls', () => {
    const container = {
      clientWidth: 800,
      clientHeight: 600,
      appendChild: vi.fn(),
    };
    const result = createScene(container);

    expect(result).toHaveProperty('scene');
    expect(result).toHaveProperty('camera');
    expect(result).toHaveProperty('renderer');
    expect(result).toHaveProperty('controls');
  });

  it('appends renderer canvas to container', () => {
    const container = {
      clientWidth: 800,
      clientHeight: 600,
      appendChild: vi.fn(),
    };
    createScene(container);

    expect(container.appendChild).toHaveBeenCalledOnce();
  });

  it('sets up directional and ambient lights', () => {
    const container = {
      clientWidth: 800,
      clientHeight: 600,
      appendChild: vi.fn(),
    };
    const { scene } = createScene(container);

    const lights = scene.children.filter(
      (c) => c.isLight
    );
    expect(lights.length).toBeGreaterThanOrEqual(2);
  });
});
