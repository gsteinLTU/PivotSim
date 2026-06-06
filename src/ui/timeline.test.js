import { describe, it, expect, vi } from 'vitest';
import { createTimeline } from './timeline.js';

function makeContainer() {
  return document.createElement('div');
}

describe('createTimeline', () => {
  it('returns required methods', () => {
    const tl = createTimeline(makeContainer(), {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    expect(typeof tl.setState).toBe('function');
    expect(typeof tl.updateProgress).toBe('function');
    expect(typeof tl.setResult).toBe('function');
    expect(typeof tl.updatePlayhead).toBe('function');
    expect(typeof tl.setPlayState).toBe('function');
  });

  it('idle state shows a solve button', () => {
    const container = makeContainer();
    createTimeline(container, {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    const btn = container.querySelector('button');
    expect(btn.textContent.toLowerCase()).toMatch(/solve/);
  });

  it('onSolve fires when solve button clicked', () => {
    const container = makeContainer();
    const onSolve = vi.fn();
    createTimeline(container, {
      onSolve, onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    container.querySelector('button').click();
    expect(onSolve).toHaveBeenCalledOnce();
  });

  it('solving state shows cancel button', () => {
    const container = makeContainer();
    const tl = createTimeline(container, {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    tl.setState('solving');
    const btns = Array.from(container.querySelectorAll('button'));
    expect(btns.some(b => b.textContent.includes('Cancel'))).toBe(true);
  });

  it('onCancel fires when cancel button clicked', () => {
    const container = makeContainer();
    const onCancel = vi.fn();
    const tl = createTimeline(container, {
      onSolve: vi.fn(), onCancel, onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    tl.setState('solving');
    const cancel = Array.from(container.querySelectorAll('button')).find(b => b.textContent.includes('Cancel'));
    cancel.click();
    expect(onCancel).toHaveBeenCalledOnce();
  });

  it('done state with fits=true shows success banner', () => {
    const container = makeContainer();
    const tl = createTimeline(container, {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    tl.setState('done', { fits: true, tightestIndex: 0, poses: [
      { x:0,y:0,z:0,yaw:0,pitch:0,roll:0 },
      { x:0,y:0,z:1,yaw:0,pitch:0,roll:0 },
    ], segmentTimes: [2], totalTime: 2 });
    expect(container.textContent).toMatch(/fits/i);
    expect(container.textContent).not.toMatch(/may still/i);
  });

  it('done state with fits=false shows warning banner', () => {
    const container = makeContainer();
    const tl = createTimeline(container, {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    tl.setState('done', { fits: false, tightestIndex: 0, poses: [
      { x:0,y:0,z:0,yaw:0,pitch:0,roll:0 },
      { x:0,y:0,z:1,yaw:0,pitch:0,roll:0 },
    ], segmentTimes: [2], totalTime: 2 });
    expect(container.textContent).toMatch(/may still/i);
    expect(container.textContent).not.toMatch(/✓/);
  });

  it('setState idle resets to solve button', () => {
    const container = makeContainer();
    const tl = createTimeline(container, {
      onSolve: vi.fn(), onCancel: vi.fn(), onPlayheadChange: vi.fn(),
      onPlayToggle: vi.fn(), onSpeedChange: vi.fn(), onReset: vi.fn(),
    });
    tl.setState('solving');
    tl.setState('idle');
    const btn = container.querySelector('button');
    expect(btn.textContent.toLowerCase()).toMatch(/solve/);
  });
});
