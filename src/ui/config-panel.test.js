import { describe, it, expect, vi } from 'vitest';
import { createConfigPanel } from './config-panel.js';
import { DEFAULTS, BOX_DEFAULTS, BOX_POSE_DEFAULTS } from '../defaults.js';

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

  it('getBoxDims returns default box dimensions', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    const dims = panel.getBoxDims();
    expect(dims.length).toBe(2.0);
    expect(dims.width).toBe(0.8);
    expect(dims.height).toBe(0.5);
  });

  it('getBoxPose returns default box pose', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    const pose = panel.getBoxPose();
    expect(pose.y).toBeCloseTo(0.25);
    expect(pose.z).toBeCloseTo(-1.0);
    expect(pose.yaw).toBe(0);
  });

  it('onBoxDimsChange and onBoxPoseChange are functions', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    expect(typeof panel.onBoxDimsChange).toBe('function');
    expect(typeof panel.onBoxPoseChange).toBe('function');
  });

  it('onBoxDimsChange is called after a box dim input changes', async () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const cb = vi.fn();
    panel.onBoxDimsChange(cb);

    // Box dim inputs follow all the stairwell inputs; find by label text
    const labels = Array.from(container.querySelectorAll('label'));
    const lengthLabel = labels.find((l) => l.textContent === 'Length (m)');
    const input = lengthLabel.nextElementSibling;
    input.value = '1.5';
    input.dispatchEvent(new Event('input'));

    await new Promise((resolve) => setTimeout(resolve, 150));
    expect(cb).toHaveBeenCalledWith(expect.objectContaining({ length: 1.5 }));
  });

  it('lock() disables all inputs and selects', () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    panel.lock();
    const inputs = Array.from(container.querySelectorAll('input, select'));
    expect(inputs.length).toBeGreaterThan(0);
    expect(inputs.every(el => el.disabled)).toBe(true);
  });

  it('unlock() re-enables all inputs and selects', () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    panel.lock();
    panel.unlock();
    const inputs = Array.from(container.querySelectorAll('input, select'));
    expect(inputs.every(el => !el.disabled)).toBe(true);
  });
});
