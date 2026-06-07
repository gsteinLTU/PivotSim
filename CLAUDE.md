# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
npm run dev          # Start Vite dev server (http://localhost:5173)
npm test             # Run all tests once
npm run test:watch   # Run tests in watch mode
npm run build        # Production build
npx vitest run src/geometry/stairwell.test.js  # Run a single test file
```

## Architecture

PivotSim is a browser-based 3D tool for simulating moving furniture up staircases. Built with Vite + Three.js + Vitest. No TypeScript.

**Module map:**

| File | Exports | Purpose |
|------|---------|---------|
| `src/defaults.js` | `DEFAULTS`, `BOX_DEFAULTS`, `BOX_POSE_DEFAULTS` | Single source of truth for all default parameter values |
| `src/geometry/stairwell.js` | `buildStairwell(params)` | Builds Three.js meshes + collision quads from params |
| `src/geometry/box.js` | `createBoxMesh`, `updateBoxMeshPose`, `computeOBB`, `getOBBCorners`, `getHalfExtents` | Box mesh creation and OBB math (pure, no Three.js in math fns) |
| `src/solver/collision.js` | `testOBBvsQuad`, `checkCollisions` | SAT collision detection — pure JS, no Three.js dependency |
| `src/solver/utils.js` | `euclideanDelta`, `angularDelta`, `segmentDuration`, `lerpPose`, `applyRotationPropagation`, `computeOBBFromPose`, `MAX_LINEAR_SPEED`, `MAX_ANGULAR_SPEED` | Pure pose-math shared across all planners |
| `src/solver/path.js` | `buildCenterline`, `getEndpoints`, `buildContainmentOBBs`, `getSegmentBoundaries` | Builds stairwell centerline polyline, containment OBBs, and segment boundary poses |
| `src/solver/gateway.js` | `findGatewayConfigs`, `bestGatewayConfig` | Sweeps orientation grid at segment transitions to find collision-free gateway poses |
| `src/solver/context.js` | `buildPlannerContext(stairwellParams, boxDims)` | Assembles all planner inputs (quads, centerline, OBBs, start/end poses) into one context object |
| `src/solver/trajectory.js` | `buildTrajectory(poses)` | Converts a pose array into a timed trajectory |
| `src/solver/planners/sa.js` | `DEFAULT_WEIGHTS`, `evalSegment`, `saPlanner` | Simulated-annealing planner over 6-DOF box poses |
| `src/solver/planners/rrt-connect.js` | `DEFAULTS`, `rrtPlanner` | RRT-Connect planner; samples gateway-biased poses and grows bidirectional trees |
| `src/solver/worker.js` | — | Web Worker wrapper; dispatches to `saPlanner` or `rrtPlanner` via `plannerType`; accepts `start`/`cancel` messages |
| `src/viewer/scene.js` | `createScene(container)` | Sets up Three.js scene, camera, renderer, OrbitControls |
| `src/viewer/debug.js` | `buildQuadDebug(quads)` | Builds colored wireframe + normal arrows for collision quad visualization |
| `src/ui/config-panel.js` | `createConfigPanel(container, params, onChange)` | Renders all parameter inputs, debounced onChange |
| `src/ui/timeline.js` | `createTimeline(container, callbacks)` | Timeline UI: solve trigger, playback controls, scrubber |
| `src/main.js` | — | Entry point: wires everything together |

**OBB format** (used by collision solver):
```js
{ center: [x,y,z], axes: [[x,y,z],[x,y,z],[x,y,z]], halfExtents: [hWidth, hHeight, hLength] }
```
Axes are unit vectors (local X/Y/Z in world space). `halfExtents` ordering matches `axes`: `[0]=width, [1]=height, [2]=length`.

**Collision quad format** (used by the solver):
```js
{ type: string, segment: string, vertices: [[x,y,z], [x,y,z], [x,y,z], [x,y,z]], normal: [nx, ny, nz] }
```
Types: `tread`, `riser`, `wall-left`, `wall-right`, `ceiling`, `floor`
Segments: `stair`, `bottom-hall`, `top-hall`

**Stairwell coordinate system:** stairs run along +Z, rise along +Y, width along X. Bottom hallway attaches at (0,0,0); top hallway attaches at (0, totalRise, totalRun).

**Hallway rotation direction:** `hallGroup.rotation.y = -turnRad` for bottom hallway. Negative angle is correct — Three.js Y rotation is CCW from above, so −90° maps local −Z to +X ("right turn"). `+turnRad` was a bug.

**Surface mesh tagging:** Transparent wall/ceiling meshes are tagged `mesh.userData.isSurface = true`. The ceiling visibility toggle in `main.js` uses this tag (not opacity) to show/hide surfaces.

**Test environment:** jsdom (configured in `vite.config.js`). Three.js geometry constructors work without mocking. Only `WebGLRenderer` and `OrbitControls` need mocks — see `src/viewer/scene.test.js` for the pattern. The `OrbitControls` mock target needs both `.set()` and `.copy()` since `rebuildStairwell` calls `controls.target.copy(center)`.

## Phased Roadmap

- **Phase 0+1** ✅ — Scaffold + stairwell geometry visualizer
- **Phase 1.5** ✅ — Box model (`src/geometry/box.js`) + SAT collision detector (`src/solver/collision.js`); manual box placement with real-time collision feedback
- **Phase 2** ✅ — Trajectory solver (`src/solver/trajectory.js`) + timeline UI (`src/ui/timeline.js`)
- **Phase 3** ✅ — Dual-planner architecture: SA refactored to `planners/sa.js`; RRT-Connect planner added (`planners/rrt-connect.js`); gateway pose pre-computation (`gateway.js`); shared planner context (`context.js`)

Full spec: `docs/superpowers/specs/2026-06-05-pivotsim-design.md`
