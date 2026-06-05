import { describe, it, expect, vi } from 'vitest';
import { createConfigPanel } from './config-panel.js';
import { DEFAULTS } from '../defaults.js';

describe('createConfigPanel', () => {
  function makeContainer() {
    const el = document.createElement('div');
    return el;
  }

  it('returns an object with a getParams method', () => {
    const container = makeContainer();
    const onChange = vi.fn();
    const panel = createConfigPanel(container, { ...DEFAULTS }, onChange);
    expect(typeof panel.getParams).toBe('function');
  });

  it('getParams returns current parameter values', () => {
    const container = makeContainer();
    const onChange = vi.fn();
    const panel = createConfigPanel(container, { ...DEFAULTS }, onChange);
    const params = panel.getParams();
    expect(params.stairWidth).toBe(DEFAULTS.stairWidth);
    expect(params.numSteps).toBe(DEFAULTS.numSteps);
  });
});
