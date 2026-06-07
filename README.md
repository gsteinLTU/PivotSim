# PivotSim

A browser-based 3D tool for simulating whether large furniture can be moved up a staircase. Model your stairway geometry, enter the furniture dimensions, and let the solver find a collision-free path — or tell you one doesn't exist.

**[Try it live →](https://gsteinLTU.github.io/PivotSim/)**

## What it does

- Parametric stairwell geometry (steps, width, hallway turn angle, ceiling height)
- 3D furniture box with full 6-DOF pose control
- Real-time SAT collision detection against stair treads, risers, walls, and ceiling
- Automated trajectory solver using SA (simulated annealing) or RRT-Connect
- Scrubable timeline playback of the solved path

## Development

```bash
npm install
npm run dev       # http://localhost:5173
npm test          # run all tests
npm run build     # production build → dist/
```

Built with [Vite](https://vite.dev), [Three.js](https://threejs.org), and [Vitest](https://vitest.dev). No TypeScript.
