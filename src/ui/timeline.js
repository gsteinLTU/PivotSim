const BTN = 'background:transparent;border:none;cursor:pointer;font-size:14px;';
const MONO = 'font-family:monospace;font-size:12px;';

function fmt(s) {
  const m = Math.floor(s / 60), sec = Math.floor(s % 60);
  return `${m}:${sec.toString().padStart(2, '0')}`;
}

export function createTimeline(container, callbacks) {
  const { onSolve, onCancel, onPlayheadChange, onPlayToggle, onSpeedChange, onReset } = callbacks;

  let totalTime = 0;

  function renderIdle() {
    container.innerHTML = `
      <div style="display:flex;align-items:center;height:100%;padding:0 20px;">
        <button id="tl-solve" style="background:#64ffda;color:#0a0a1a;border:none;
          border-radius:4px;padding:8px 24px;font-size:14px;font-weight:bold;cursor:pointer;">
          ▶ SOLVE
        </button>
      </div>`;
    container.querySelector('#tl-solve').addEventListener('click', onSolve);
  }

  function renderSolving() {
    container.innerHTML = `
      <div style="display:flex;align-items:center;gap:12px;height:100%;padding:0 16px;">
        <button id="tl-cancel" style="color:#ff4444;border:1px solid #ff4444;
          border-radius:4px;padding:4px 12px;background:transparent;cursor:pointer;">
          ✕ Cancel
        </button>
        <span id="tl-temp" style="${MONO}color:#64ffda;">T=5.000</span>
        <span id="tl-iter" style="${MONO}color:#aaa;">0 / 50,000</span>
        <div style="flex:1;height:6px;background:#1a2a3a;border-radius:3px;overflow:hidden;">
          <div id="tl-ebar" style="height:100%;width:0%;background:#64ffda;border-radius:3px;
            transition:width 0.4s;"></div>
        </div>
      </div>`;
    container.querySelector('#tl-cancel').addEventListener('click', onCancel);
  }

  function renderDone(data) {
    const { fits, poses, segmentTimes: st, totalTime: tt, tightestIndex } = data ?? {};
    totalTime = tt ?? 0;

    const banner = fits === true
      ? `<span style="color:#22ff88;">✓ Box fits!</span>`
      : fits === false
        ? `<span style="color:#ffaa00;">~ Best trajectory found — may still collide</span>`
        : `<span style="color:#888;">~ Canceled — partial result</span>`;

    let keyframeHTML = '';
    if (poses && st && totalTime > 0) {
      let elapsed = 0;
      for (let i = 0; i < poses.length; i++) {
        const pct = (elapsed / totalTime) * 100;
        const color = i === tightestIndex ? '#ff4444'
                    : (st[i - 1] ?? 0) < 0.05 ? '#ffaa00' : null;
        if (color) {
          keyframeHTML += `<div style="position:absolute;left:${pct}%;top:-5px;
            width:8px;height:8px;background:${color};border-radius:50%;
            transform:translateX(-50%);pointer-events:none;"></div>`;
        }
        if (i < st.length) elapsed += st[i];
      }
    }

    container.innerHTML = `
      <div id="tl-banner" style="padding:2px 16px;font-size:12px;">${banner}</div>
      <div style="display:flex;align-items:center;gap:6px;padding:2px 12px;">
        <button id="tl-reset" title="Re-solve" style="${BTN}color:#aaa;">↺</button>
        <button id="tl-prev"  title="Back to start" style="${BTN}color:#aaa;">⏮</button>
        <button id="tl-play" style="${BTN}color:#64ffda;">▶</button>
        <div style="flex:1;position:relative;">
          <input type="range" id="tl-scrubber" min="0" max="1000" value="0" step="1"
            style="width:100%;cursor:pointer;accent-color:#64ffda;">
          <div id="tl-kf" style="position:absolute;top:0;left:0;width:100%;height:100%;
            pointer-events:none;">${keyframeHTML}</div>
        </div>
        <span id="tl-time" style="${MONO}color:#aaa;white-space:nowrap;">
          0:00 / ${fmt(totalTime)}
        </span>
        <select id="tl-speed" style="background:#0d1b2a;color:#aaa;border:1px solid #334;
          border-radius:3px;font-size:11px;padding:2px 4px;">
          <option value="0.25">¼×</option>
          <option value="0.5">½×</option>
          <option value="1" selected>1×</option>
          <option value="2">2×</option>
        </select>
      </div>`;

    container.querySelector('#tl-reset').addEventListener('click', () => {
      setState('idle');
      onReset?.();
    });
    container.querySelector('#tl-prev').addEventListener('click', () => {
      onPlayheadChange?.(0);
      onPlayToggle?.(false);
    });
    container.querySelector('#tl-play').addEventListener('click', () => {
      const btn = container.querySelector('#tl-play');
      const nowPlaying = btn.textContent === '▶';
      btn.textContent = nowPlaying ? '⏸' : '▶';
      onPlayToggle?.(nowPlaying);
    });
    container.querySelector('#tl-scrubber').addEventListener('input', (e) => {
      const secs = (Number(e.target.value) / 1000) * totalTime;
      onPlayheadChange?.(secs);
      const el = container.querySelector('#tl-time');
      if (el) el.textContent = `${fmt(secs)} / ${fmt(totalTime)}`;
    });
    container.querySelector('#tl-speed').addEventListener('change', (e) => {
      onSpeedChange?.(Number(e.target.value));
    });
  }

  function setState(state, data) {
    if (state === 'idle')         renderIdle();
    else if (state === 'solving') renderSolving();
    else if (state === 'done')    renderDone(data);
  }

  function updateProgress({ energy, temperature, iteration }) {
    const t = container.querySelector('#tl-temp');
    const i = container.querySelector('#tl-iter');
    const b = container.querySelector('#tl-ebar');
    if (t) t.textContent = `T=${temperature.toFixed(3)}`;
    if (i) i.textContent = `${iteration.toLocaleString()} / 50,000`;
    if (b) b.style.width = `${Math.min(100, (iteration / 50000) * 100)}%`;
  }

  function setResult(data) {
    setState('done', data);
  }

  function updatePlayhead(seconds) {
    const scrubber = container.querySelector('#tl-scrubber');
    if (scrubber && totalTime > 0) {
      scrubber.value = Math.round((seconds / totalTime) * 1000);
    }
    const el = container.querySelector('#tl-time');
    if (el) el.textContent = `${fmt(seconds)} / ${fmt(totalTime)}`;
  }

  function setPlayState(playing) {
    const btn = container.querySelector('#tl-play');
    if (btn) btn.textContent = playing ? '⏸' : '▶';
  }

  renderIdle();
  return { setState, updateProgress, setResult, updatePlayhead, setPlayState };
}
