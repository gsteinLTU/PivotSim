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

    // Box dim inputs are in a flex row; find by label text then get input from the row
    const labels = Array.from(container.querySelectorAll('label'));
    const lengthLabel = labels.find((l) => l.textContent === 'Length');
    const row = lengthLabel.nextElementSibling;
    const input = row.querySelector('input[type="number"]');
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

describe('stairwell config unit dropdowns', () => {
  beforeEach(() => {
    localStorage.clear();
  });

  it('length fields have a unit select with options m, ft, in', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    // stairWidth is the first length field — find its wrapper
    const selects = Array.from(container.querySelectorAll('select'));
    // turn selects (bottomHallwayTurn, topHallwayTurn) have value options like '0','90','-90'
    // unit selects have 'm','ft','in'
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    expect(unitSelects.length).toBeGreaterThan(0);
    const opts = Array.from(unitSelects[0].options).map((o) => o.value);
    expect(opts).toEqual(['m', 'ft', 'in']);
  });

  it('changing unit select converts the displayed value', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    const firstSelect = unitSelects[0]; // stairWidth unit select
    const numberInput = firstSelect.closest('div').querySelector('input[type="number"]');

    // default stairWidth is 1.0 m
    expect(Number(numberInput.value)).toBeCloseTo(1.0);

    firstSelect.value = 'in';
    firstSelect.dispatchEvent(new Event('change'));

    // 1.0 m = 39.3701 in
    expect(Number(numberInput.value)).toBeCloseTo(39.37, 1);
  });

  it('onChange callback receives meters regardless of display unit', async () => {
    const container = document.createElement('div');
    const onChange = vi.fn();
    createConfigPanel(container, { ...DEFAULTS }, onChange);

    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    const firstSelect = unitSelects[0];
    const numberInput = firstSelect.closest('div').querySelector('input[type="number"]');

    firstSelect.value = 'in';
    firstSelect.dispatchEvent(new Event('change'));

    // type 39.3701 in (= 1.0 m)
    numberInput.value = '39.3701';
    numberInput.dispatchEvent(new Event('input'));

    await new Promise((r) => setTimeout(r, 150));
    expect(onChange).toHaveBeenCalledWith(
      expect.objectContaining({ stairWidth: expect.closeTo(1.0, 3) })
    );
  });

  it('persists unit selection to localStorage', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());

    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    unitSelects[0].value = 'ft';
    unitSelects[0].dispatchEvent(new Event('change'));

    const stored = JSON.parse(localStorage.getItem('pivotsim_unit_prefs'));
    expect(stored.stairWidth).toBe('ft');
  });

  it('restores unit selection from localStorage on init', () => {
    localStorage.setItem('pivotsim_unit_prefs', JSON.stringify({ stairWidth: 'in' }));
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());

    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    expect(unitSelects[0].value).toBe('in');
    const numberInput = unitSelects[0].closest('div').querySelector('input[type="number"]');
    // stairWidth default 1.0 m shown in inches
    expect(Number(numberInput.value)).toBeCloseTo(39.37, 1);
  });
});

describe('box dimensions unit dropdowns', () => {
  beforeEach(() => { localStorage.clear(); });

  it('box dimension inputs have unit selects', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    // 7 stairwell length fields + 3 box dims = at least 10
    expect(unitSelects.length).toBeGreaterThanOrEqual(10);
  });

  it('onBoxDimsChange callback receives meters', async () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const cb = vi.fn();
    panel.onBoxDimsChange(cb);

    // Find Box Dimensions section — length label now reads 'Length' (no unit suffix)
    const labels = Array.from(container.querySelectorAll('label'));
    const lengthLabel = labels.find((l) => l.textContent === 'Length');
    const row = lengthLabel.nextElementSibling;
    const unitSel = row.querySelector('select');
    const input = row.querySelector('input[type="number"]');

    unitSel.value = 'in';
    unitSel.dispatchEvent(new Event('change'));

    // type 19.685 in ≈ 0.5 m
    input.value = '19.685';
    input.dispatchEvent(new Event('input'));

    await new Promise((r) => setTimeout(r, 150));
    expect(cb).toHaveBeenCalledWith(
      expect.objectContaining({ length: expect.closeTo(0.5, 2) })
    );
  });
});

describe('box pose unit dropdowns', () => {
  beforeEach(() => { localStorage.clear(); });

  it('x/y/z pose inputs have unit selects; yaw/pitch/roll do not', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const selects = Array.from(container.querySelectorAll('select'));
    const unitSelects = selects.filter((s) =>
      Array.from(s.options).some((o) => o.value === 'in')
    );
    // 7 stairwell + 3 box dims + 3 pose (x,y,z) = 13
    expect(unitSelects.length).toBe(13);
  });

  it('onBoxPoseChange callback receives meters for x/y/z', async () => {
    const container = document.createElement('div');
    const panel = createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const cb = vi.fn();
    panel.onBoxPoseChange(cb);

    const labels = Array.from(container.querySelectorAll('label'));
    const xLabel = labels.find((l) => l.textContent === 'X');
    const row = xLabel.nextElementSibling;
    const unitSel = row.querySelector('select');
    const input = row.querySelector('input[type="number"]');

    unitSel.value = 'in';
    unitSel.dispatchEvent(new Event('change'));

    // type 3.937 in ≈ 0.1 m
    input.value = '3.937';
    input.dispatchEvent(new Event('input'));

    await new Promise((r) => setTimeout(r, 150));
    expect(cb).toHaveBeenCalledWith(
      expect.objectContaining({ x: expect.closeTo(0.1, 2) })
    );
  });

  it('yaw input has no unit select', () => {
    const container = document.createElement('div');
    createConfigPanel(container, { ...DEFAULTS }, vi.fn());
    const labels = Array.from(container.querySelectorAll('label'));
    const yawLabel = labels.find((l) => l.textContent === 'Yaw');
    // Yaw input is a plain number input sibling, not in a flex row with a select
    const next = yawLabel.nextElementSibling;
    expect(next.tagName).toBe('INPUT');
    expect(next.type).toBe('number');
  });
});
