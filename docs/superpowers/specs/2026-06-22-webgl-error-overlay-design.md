# WebGL Error Overlay — Design Spec

**Date:** 2026-06-22

## Problem

When a browser cannot create a WebGL context (hardware acceleration disabled, unsupported GPU, or software rendering fallback unavailable), PivotSim displays a blank page. The Three.js renderer either throws silently or produces a broken renderer with no visible feedback to the user.

## Goal

Show a clear, full-screen error message when WebGL is unavailable, instead of a blank or partially-rendered page.

## Approach

Pre-flight WebGL detection at the top of `main.js`, before any Three.js or DOM setup runs. If WebGL is unavailable, inject a full-screen overlay and halt module execution.

## Detection

```js
const testCanvas = document.createElement('canvas');
const hasWebGL = testCanvas.getContext('webgl2') || testCanvas.getContext('webgl');
```

Probing both `webgl2` and `webgl` covers the full range of supported contexts. If both return null, WebGL is unavailable.

## Overlay

A `position:fixed; inset:0` div injected into `document.body`. Content is centered. Execution halts via `throw` in module scope after injection.

**Content:**
- Title: "WebGL Not Available"
- Body: "PivotSim requires WebGL to render the 3D view."
- Suggestions (short list):
  - Enable hardware acceleration in your browser settings
  - Try Chrome, Firefox, or Edge
  - Update your GPU drivers

**Styling:** Inline styles only — no new CSS file. Dark palette to match the app:
- Background: `#0a0a1a`
- Text: `#e0e0e0`
- Title uses monospace font to match the existing clearance readout style

## Scope

- **Changed:** `src/main.js` — ~15 lines added at the top
- **Unchanged:** `src/viewer/scene.js`, `index.html`, all other files

## Out of Scope

- Graceful degradation or software renderer fallback
- Localisation
- Logging / analytics
