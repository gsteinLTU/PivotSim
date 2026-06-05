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

  it('onCeilingToggle calls callback with checkbox state on change', () => {
    const container = makeContainer();
    const onChange = vi.fn();
    const panel = createConfigPanel(container, { ...DEFAULTS }, onChange);
    const callback = vi.fn();
    panel.onCeilingToggle(callback);

    // Find ceiling checkbox by label text (robust to new checkboxes being added)
    const labels = Array.from(container.querySelectorAll('label'));
    const ceilLabel = labels.find((l) => l.textContent.includes('Show Ceiling'));
    const ceilCheckbox = ceilLabel.querySelector('input[type="checkbox"]');
    ceilCheckbox.checked = false;
    ceilCheckbox.dispatchEvent(new Event('change'));
    expect(callback).toHaveBeenCalledWith(false);
  });

  it('onQuadDebugToggle calls callback with checkbox state on change', () => {
    const container = makeContainer();
    const onChange = vi.fn();
    const panel = createConfigPanel(container, { ...DEFAULTS }, onChange);
    const callback = vi.fn();
    panel.onQuadDebugToggle(callback);

    const labels = Array.from(container.querySelectorAll('label'));
    const quadLabel = labels.find((l) => l.textContent.includes('Show Collision Quads'));
    const quadCheckbox = quadLabel.querySelector('input[type="checkbox"]');
    quadCheckbox.checked = true;
    quadCheckbox.dispatchEvent(new Event('change'));
    expect(callback).toHaveBeenCalledWith(true);
  });

  it('onChange is called after a number input changes', async () => {
    const container = makeContainer();
    const onChange = vi.fn();
    createConfigPanel(container, { ...DEFAULTS }, onChange);

    const numberInput = container.querySelector('input[type="number"]');
    numberInput.value = '1.5';
    numberInput.dispatchEvent(new Event('input'));

    await new Promise((resolve) => setTimeout(resolve, 150));
    expect(onChange).toHaveBeenCalled();
  });
});
