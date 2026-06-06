import { describe, it, expect, vi, beforeEach } from 'vitest';
import { createConfigPanel, toMeters, fromMeters, loadUnitPrefs } from './config-panel.js';
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
    expect(dims.length).toBe(BOX_DEFAULTS.length);
    expect(dims.width).toBe(BOX_DEFAULTS.width);
    expect(dims.height).toBe(BOX_DEFAULTS.height);
  });

  it('getBoxPose returns default box pose', () => {
    const panel = createConfigPanel(document.createElement('div'), { ...DEFAULTS }, vi.fn());
    const pose = panel.getBoxPose();
    expect(pose.y).toBeCloseTo(BOX_POSE_DEFAULTS.y);
    expect(pose.z).toBeCloseTo(BOX_POSE_DEFAULTS.z);
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

describe('toMeters', () => {
  it('returns value unchanged for m', () => {
    expect(toMeters(1.5, 'm')).toBeCloseTo(1.5);
  });
  it('converts ft to m', () => {
    expect(toMeters(1, 'ft')).toBeCloseTo(0.3048);
  });
  it('converts in to m', () => {
    expect(toMeters(84, 'in')).toBeCloseTo(2.1336);
  });
  it('converts 12 in to same as 1 ft', () => {
    expect(toMeters(12, 'in')).toBeCloseTo(toMeters(1, 'ft'));
  });
});

describe('fromMeters', () => {
  it('returns value unchanged for m', () => {
    expect(fromMeters(1.5, 'm')).toBeCloseTo(1.5);
  });
  it('converts m to ft', () => {
    expect(fromMeters(0.3048, 'ft')).toBeCloseTo(1.0);
  });
  it('converts m to in', () => {
    expect(fromMeters(0.0254, 'in')).toBeCloseTo(1.0);
  });
  it('round-trips m → in → m', () => {
    expect(toMeters(fromMeters(1.2, 'in'), 'in')).toBeCloseTo(1.2);
  });
});

describe('loadUnitPrefs', () => {
  const KEYS = ['stairWidth', 'risePerStep', 'x'];

  beforeEach(() => {
    localStorage.clear();
  });

  it('returns "m" for all keys when localStorage is empty', () => {
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs).toEqual({ stairWidth: 'm', risePerStep: 'm', x: 'm' });
  });

  it('returns stored valid unit values', () => {
    localStorage.setItem('pivotsim_unit_prefs', JSON.stringify({ stairWidth: 'in', risePerStep: 'ft', x: 'm' }));
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs).toEqual({ stairWidth: 'in', risePerStep: 'ft', x: 'm' });
  });

  it('falls back to "m" for unknown unit values', () => {
    localStorage.setItem('pivotsim_unit_prefs', JSON.stringify({ stairWidth: 'cm', risePerStep: 'yards' }));
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs.stairWidth).toBe('m');
    expect(prefs.risePerStep).toBe('m');
  });

  it('falls back to "m" for missing keys', () => {
    localStorage.setItem('pivotsim_unit_prefs', JSON.stringify({ stairWidth: 'in' }));
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs.risePerStep).toBe('m');
    expect(prefs.x).toBe('m');
  });

  it('ignores unknown keys in stored data', () => {
    localStorage.setItem('pivotsim_unit_prefs', JSON.stringify({ unknownField: 'in' }));
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs).not.toHaveProperty('unknownField');
    expect(prefs.stairWidth).toBe('m');
  });

  it('falls back to all "m" when localStorage contains invalid JSON', () => {
    localStorage.setItem('pivotsim_unit_prefs', 'not-json{{{');
    const prefs = loadUnitPrefs(KEYS);
    expect(prefs).toEqual({ stairWidth: 'm', risePerStep: 'm', x: 'm' });
  });
});
