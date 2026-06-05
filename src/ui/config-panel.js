const FIELD_DEFS = [
  { key: 'stairWidth', label: 'Stair Width (m)', type: 'number', min: 0.5, max: 3, step: 0.05 },
  { key: 'numSteps', label: 'Number of Steps', type: 'number', min: 1, max: 30, step: 1 },
  { key: 'risePerStep', label: 'Rise per Step (m)', type: 'number', min: 0.1, max: 0.4, step: 0.01 },
  { key: 'runPerStep', label: 'Run per Step (m)', type: 'number', min: 0.15, max: 0.5, step: 0.01 },
  { key: 'bottomHallwayWidth', label: 'Bottom Hallway Width (m)', type: 'number', min: 0.5, max: 3, step: 0.05 },
  { key: 'bottomHallwayTurn', label: 'Bottom Hallway Turn', type: 'select', options: [
    { value: 0, label: 'Straight (0°)' },
    { value: 90, label: 'Right (90°)' },
    { value: -90, label: 'Left (-90°)' },
  ]},
  { key: 'topHallwayWidth', label: 'Top Hallway Width (m)', type: 'number', min: 0.5, max: 3, step: 0.05 },
  { key: 'topHallwayTurn', label: 'Top Hallway Turn', type: 'select', options: [
    { value: 0, label: 'Straight (0°)' },
    { value: 90, label: 'Right (90°)' },
    { value: -90, label: 'Left (-90°)' },
  ]},
  { key: 'ceilingHeight', label: 'Ceiling Height (m)', type: 'number', min: 1.8, max: 4, step: 0.05 },
  { key: 'slopedCeiling', label: 'Sloped Ceiling', type: 'checkbox' },
  { key: 'hallwayLength', label: 'Hallway Length (m)', type: 'number', min: 1, max: 6, step: 0.1 },
];

export function createConfigPanel(container, initialParams, onChange) {
  const params = { ...initialParams };
  let debounceTimer = null;

  const title = document.createElement('h2');
  title.textContent = 'Stairwell Config';
  title.style.cssText = 'margin-bottom:16px; font-size:18px; color:#64ffda;';
  container.appendChild(title);

  const inputs = {};

  for (const def of FIELD_DEFS) {
    const wrapper = document.createElement('div');
    wrapper.style.cssText = 'margin-bottom:12px;';

    const label = document.createElement('label');
    label.textContent = def.label;
    label.style.cssText = 'display:block; font-size:12px; margin-bottom:4px; color:#aaa;';
    wrapper.appendChild(label);

    let input;

    if (def.type === 'select') {
      input = document.createElement('select');
      for (const opt of def.options) {
        const option = document.createElement('option');
        option.value = opt.value;
        option.textContent = opt.label;
        if (Number(opt.value) === params[def.key]) option.selected = true;
        input.appendChild(option);
      }
      input.addEventListener('change', () => {
        params[def.key] = Number(input.value);
        scheduleUpdate();
      });
    } else if (def.type === 'checkbox') {
      input = document.createElement('input');
      input.type = 'checkbox';
      input.checked = params[def.key];
      input.addEventListener('change', () => {
        params[def.key] = input.checked;
        scheduleUpdate();
      });
    } else {
      input = document.createElement('input');
      input.type = 'number';
      input.value = params[def.key];
      input.min = def.min;
      input.max = def.max;
      input.step = def.step;
      input.style.cssText = 'width:100%; padding:4px 8px; background:#0d1b2a; color:#e0e0e0; border:1px solid #334; border-radius:4px;';
      input.addEventListener('input', () => {
        params[def.key] = Number(input.value);
        scheduleUpdate();
      });
    }

    inputs[def.key] = input;
    wrapper.appendChild(input);
    container.appendChild(wrapper);
  }

  function scheduleUpdate() {
    clearTimeout(debounceTimer);
    debounceTimer = setTimeout(() => onChange(params), 100);
  }

  // Ceiling visibility toggle
  const separator = document.createElement('hr');
  separator.style.cssText = 'border-color:#334; margin:16px 0;';
  container.appendChild(separator);

  const vizTitle = document.createElement('h3');
  vizTitle.textContent = 'Display';
  vizTitle.style.cssText = 'font-size:14px; color:#64ffda; margin-bottom:8px;';
  container.appendChild(vizTitle);

  const ceilToggle = document.createElement('label');
  ceilToggle.style.cssText = 'display:flex; align-items:center; gap:8px; font-size:12px; color:#aaa; cursor:pointer;';
  const ceilCheck = document.createElement('input');
  ceilCheck.type = 'checkbox';
  ceilCheck.checked = true;
  ceilToggle.appendChild(ceilCheck);
  ceilToggle.appendChild(document.createTextNode('Show Ceiling'));
  container.appendChild(ceilToggle);

  return {
    getParams() {
      return { ...params };
    },
    onCeilingToggle(callback) {
      ceilCheck.addEventListener('change', () => callback(ceilCheck.checked));
    },
  };
}
