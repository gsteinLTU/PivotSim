import { checkCollisions } from '../collision.js';
import {
  euclideanDelta, angularDelta, lerpPose, computeOBBFromPose,
} from '../utils.js';
import { findGatewayConfigs } from '../gateway.js';

export const DEFAULTS = {
  maxIter:        150000,   // main-loop iterations (each = extend + connect)
  epsilon:        0.15,    // step cap in combined c-space metric
  connectTol:     1e-3,   // "reached" threshold (cspaceDist)
  gatewayBias:    0.25,   // probability of sampling a precomputed gateway pose
  smoothingIters: 150,    // random-shortcut smoothing iterations
  edgeResLinear:  0.025,   // m per edge-collision sample
  edgeResAngular: 0.10,   // rad per edge-collision sample
  pitchRange:     Math.PI * 0.8,  // ± limit for sampled pitch
  rollRange:      Math.PI * 0.5,   // ± limit for sampled roll
  wAng:           0.5,    // radian→meter weight in cspaceDist
  progressBatch:  50,     // iterations between onProgress calls + event-loop yield
  seed:           null,   // integer → mulberry32 PRNG; null → Math.random
};

// ── Seeded PRNG (mulberry32) ───────────────────────────────────────────────

function mulberry32(seed) {
  let s = seed >>> 0;
  return function () {
    s = (s + 0x6D2B79F5) >>> 0;
    let t = Math.imul(s ^ (s >>> 15), 1 | s);
    t     = Math.imul(t ^ (t >>> 7),  61 | t) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

// ── C-space distance metric ────────────────────────────────────────────────
// Combines meters (position) and radians (rotation) into one scalar.
// wAng=0.5 makes the planner spend rotation more freely — orientation is the
// bottleneck when squeezing through corridor pinch points.

function cspaceDist(a, b, wAng) {
  return euclideanDelta(a, b) + wAng * angularDelta(a, b);
}

// ── State / edge validity ──────────────────────────────────────────────────

function validState(pose, quads, halfExtents) {
  return checkCollisions(computeOBBFromPose(pose, halfExtents), quads).minClearance >= 0;
}

// Mirrors evalSegment's K formula so resolution matches the SA planner.
function validEdge(a, b, quads, halfExtents, cfg) {
  const dist = euclideanDelta(a, b);
  const ang  = angularDelta(a, b);
  const K    = Math.max(2,
    Math.ceil(dist / cfg.edgeResLinear),
    Math.ceil(ang  / cfg.edgeResAngular));
  for (let k = 1; k <= K; k++) {
    if (!validState(lerpPose(a, b, k / K), quads, halfExtents)) return false;
  }
  return true;
}

// ── Tree operations ────────────────────────────────────────────────────────

function newTree(rootPoses) {
  const roots = Array.isArray(rootPoses) ? rootPoses : [rootPoses];
  const nodes = roots.map(pose => ({ pose: { ...pose }, parent: -1 }));
  return { nodes };
}

function addNode(tree, pose, parentIdx) {
  tree.nodes.push({ pose: { ...pose }, parent: parentIdx });
  return tree.nodes.length - 1;
}

// Linear nearest-neighbour scan — tree sizes are modest for a single corridor.
function nearest(tree, q, wAng) {
  let bestIdx = 0, bestD = Infinity;
  for (let i = 0; i < tree.nodes.length; i++) {
    const d = cspaceDist(tree.nodes[i].pose, q, wAng);
    if (d < bestD) { bestD = d; bestIdx = i; }
  }
  return bestIdx;
}

// Steer from `from` toward `to`, capped at epsilon in cspace distance.
function steer(from, to, epsilon, wAng) {
  const d = cspaceDist(from, to, wAng);
  if (d <= epsilon) return { ...to };
  return lerpPose(from, to, epsilon / d);
}

const REACHED  = 'reached';
const ADVANCED = 'advanced';
const TRAPPED  = 'trapped';

function extend(tree, q, quads, halfExtents, cfg) {
  const nearIdx = nearest(tree, q, cfg.wAng);
  const qNear   = tree.nodes[nearIdx].pose;
  const qNew    = steer(qNear, q, cfg.epsilon, cfg.wAng);
  if (!validState(qNew, quads, halfExtents))     return { status: TRAPPED, node: -1 };
  if (!validEdge(qNear, qNew, quads, halfExtents, cfg)) return { status: TRAPPED, node: -1 };
  const idx    = addNode(tree, qNew, nearIdx);
  const status = cspaceDist(qNew, q, cfg.wAng) <= cfg.connectTol ? REACHED : ADVANCED;
  return { status, node: idx };
}

// Greedily extend tree toward q, repeating while ADVANCED.
function connect(tree, q, quads, halfExtents, cfg) {
  let result;
  do { result = extend(tree, q, quads, halfExtents, cfg); }
  while (result.status === ADVANCED);
  return result; // REACHED or TRAPPED
}

// ── Path extraction ────────────────────────────────────────────────────────

// Returns poses from root to idx (inclusive).
function pathToRoot(tree, idx) {
  const path = [];
  for (let i = idx; i !== -1; i = tree.nodes[i].parent) {
    path.push(tree.nodes[i].pose);
  }
  path.reverse();
  return path;
}

// Concatenate start-tree path and goal-tree path, dropping the duplicate
// meeting node from the goal side (fromGoal after reverse starts with it).
function extractPath(treeStart, treeGoal, conn) {
  const fromStart = pathToRoot(treeStart, conn.aIdx);
  const fromGoal  = pathToRoot(treeGoal,  conn.bIdx);
  fromGoal.reverse();           // root→meeting → meeting→root (= meeting→goal)
  return [...fromStart, ...fromGoal.slice(1)];
}

// ── Random-shortcut smoothing ──────────────────────────────────────────────

async function shortcut(path, quads, halfExtents, cfg, rng, shouldCancel) {
  let p = path;
  for (let it = 0; it < cfg.smoothingIters && p.length > 2; it++) {
    if (shouldCancel?.()) break;
    if (it > 0 && it % cfg.progressBatch === 0) {
      await new Promise(r => setTimeout(r, 0));
    }
    const i  = Math.floor(rng() * p.length);
    const j  = Math.floor(rng() * p.length);
    const lo = Math.min(i, j), hi = Math.max(i, j);
    if (hi - lo < 2) continue;
    if (validEdge(p[lo], p[hi], quads, halfExtents, cfg)) {
      p = [...p.slice(0, lo + 1), ...p.slice(hi)];
    }
  }
  return p;
}

// ── Sampling ───────────────────────────────────────────────────────────────

// Build sampling bounds once: volume-weighted OBB union, or AABB fallback.
function buildSamplingBounds(containmentOBBs, collisionQuads) {
  if (containmentOBBs && containmentOBBs.length > 0) {
    const vols  = containmentOBBs.map(o =>
      8 * o.halfExtents[0] * o.halfExtents[1] * o.halfExtents[2]);
    const total = vols.reduce((s, v) => s + v, 0);
    return { type: 'obbs', obbs: containmentOBBs, vols, total };
  }
  // AABB from quad vertices (mirrors SA's clampPose fallback)
  let bxMin = Infinity, bxMax = -Infinity;
  let byMin = Infinity, byMax = -Infinity;
  let bzMin = Infinity, bzMax = -Infinity;
  for (const quad of collisionQuads) {
    for (const [x, y, z] of quad.vertices) {
      if (x < bxMin) bxMin = x; if (x > bxMax) bxMax = x;
      if (y < byMin) byMin = y; if (y > byMax) byMax = y;
      if (z < bzMin) bzMin = z; if (z > bzMax) bzMax = z;
    }
  }
  // Guard against empty quads
  if (!isFinite(bxMin)) return { type: 'aabb', bxMin: -5, bxMax: 5, byMin: 0, byMax: 5, bzMin: -5, bzMax: 5 };
  return { type: 'aabb', bxMin, bxMax, byMin, byMax, bzMin, bzMax };
}

function samplePosition(bounds, rng) {
  if (bounds.type === 'obbs') {
    let r = rng() * bounds.total;
    let idx = 0;
    while (idx < bounds.obbs.length - 1 && r > bounds.vols[idx]) {
      r -= bounds.vols[idx];
      idx++;
    }
    const o = bounds.obbs[idx];
    const u = (rng() * 2 - 1) * o.halfExtents[0];
    const v = (rng() * 2 - 1) * o.halfExtents[1];
    const w = (rng() * 2 - 1) * o.halfExtents[2];
    return {
      x: o.center[0] + u*o.axes[0][0] + v*o.axes[1][0] + w*o.axes[2][0],
      y: o.center[1] + u*o.axes[0][1] + v*o.axes[1][1] + w*o.axes[2][1],
      z: o.center[2] + u*o.axes[0][2] + v*o.axes[1][2] + w*o.axes[2][2],
    };
  }
  return {
    x: bounds.bxMin + rng() * (bounds.bxMax - bounds.bxMin),
    y: bounds.byMin + rng() * (bounds.byMax - bounds.byMin),
    z: bounds.bzMin + rng() * (bounds.bzMax - bounds.bzMin),
  };
}

function sampleConfig(bounds, gatewayPoses, cfg, rng) {
  const r = rng();
  if (gatewayPoses.length > 0 && r < cfg.gatewayBias) {
    const g = gatewayPoses[Math.floor(rng() * gatewayPoses.length)];
    return {
      x: g.x, y: g.y, z: g.z,
      yaw:   g.yaw   + (rng() - 0.5) * 0.2,
      pitch: g.pitch + (rng() - 0.5) * 0.2,
      roll:  g.roll  + (rng() - 0.5) * 0.2,
    };
  }
  return {
    ...samplePosition(bounds, rng),
    yaw:   (rng() * 2 - 1) * Math.PI,
    pitch: (rng() * 2 - 1) * cfg.pitchRange,
    roll:  (rng() * 2 - 1) * cfg.rollRange,
  };
}

// ── Result helpers ─────────────────────────────────────────────────────────

function computeTightest(path, quads, halfExtents) {
  let tightestIndex = 0, minC = Infinity;
  for (let i = 0; i < path.length; i++) {
    const { minClearance } = checkCollisions(computeOBBFromPose(path[i], halfExtents), quads);
    if (minClearance < minC) { minC = minClearance; tightestIndex = i; }
  }
  return { tightestIndex, minC };
}

// Subsample to at most maxN evenly-spaced poses for the ghost trail.
function subsample(poses, maxN) {
  if (poses.length <= maxN) return poses;
  const out = [];
  for (let i = 0; i < maxN; i++) {
    out.push(poses[Math.floor((i / (maxN - 1)) * (poses.length - 1))]);
  }
  return out;
}

// ── Exported planner ───────────────────────────────────────────────────────

export const rrtPlanner = {
  async plan(context, config, onProgress, shouldCancel) {
    const {
      collisionQuads, halfExtents, startPose, endPose, endPoses = [],
      containmentOBBs, quadsBySegment, boundaries, centerline,
    } = context;
    const cfg = { ...DEFAULTS, ...(config ?? {}) };
    const rng = cfg.seed != null ? mulberry32(cfg.seed) : () => Math.random();

    function fallback() {
      const path = [{ ...startPose }, { ...endPose }];
      const { tightestIndex } = computeTightest(path, collisionQuads, halfExtents);
      return { poses: path, fits: false, tightestIndex };
    }

    // Abort early if start is in collision or there are no goal poses
    if (!validState(startPose, collisionQuads, halfExtents) || endPoses.length === 0) {
      return fallback();
    }

    // Precompute gateway configs for orientation-biased sampling at pinch points
    const gatewayPoses = [];
    if (quadsBySegment && boundaries && centerline) {
      const ch = centerline.ceilingHeight;
      try {
        gatewayPoses.push(...findGatewayConfigs(
          boundaries.bottomTransitionPt, ch,
          quadsBySegment['bottom-hall'], quadsBySegment.stair, halfExtents,
        ));
        gatewayPoses.push(...findGatewayConfigs(
          boundaries.topTransitionPt, ch,
          quadsBySegment.stair, quadsBySegment['top-hall'], halfExtents,
        ));
      } catch (_) { /* ignore — planner still works on pure random sampling */ }
    }

    const bounds = buildSamplingBounds(containmentOBBs, collisionQuads);

    // Two trees: start and goal. We alternate which is active (Ta/Tb) each iter.
    const treeStart = newTree(startPose);
    const treeGoal  = newTree(endPoses);
    let Ta = treeStart, Tb = treeGoal;
    let startIsA = true;  // is treeStart currently Ta?
    let connection = null; // { aIdx: index in treeStart, bIdx: index in treeGoal }
    let itersRun = 0;

    for (let iter = 0; iter < cfg.maxIter; iter++) {
      if (shouldCancel?.()) break;
      itersRun = iter + 1;

      const qRand = sampleConfig(bounds, gatewayPoses, cfg, rng);
      const ext   = extend(Ta, qRand, collisionQuads, halfExtents, cfg);

      if (ext.status !== TRAPPED) {
        const newPose = Ta.nodes[ext.node].pose;
        const con = connect(Tb, newPose, collisionQuads, halfExtents, cfg);
        if (con.status === REACHED) {
          connection = startIsA
            ? { aIdx: ext.node, bIdx: con.node }
            : { aIdx: con.node, bIdx: ext.node };
          break;
        }
      }

      // Swap roles to balance tree growth (classic RRT-Connect bidirectionality)
      [Ta, Tb] = [Tb, Ta];
      startIsA = !startIsA;

      if (itersRun % cfg.progressBatch === 0) {
        const ghostPoses = subsample(
          pathToRoot(treeStart, treeStart.nodes.length - 1), 20,
        );
        onProgress?.({
          poses:         ghostPoses,
          treeSizeStart: treeStart.nodes.length,
          treeSizeGoal:  treeGoal.nodes.length,
          iteration:     itersRun,
          maxIter:       cfg.maxIter,
          found:         connection !== null,
          plannerType:   'rrt',
        });
        await new Promise(r => setTimeout(r, 0));
      }
    }

    // Always emit a final progress update so the ghost trail shows the search result
    // (covers the common case where a path is found before the first progressBatch)
    onProgress?.({
      poses:         subsample(pathToRoot(treeStart, treeStart.nodes.length - 1), 20),
      treeSizeStart: treeStart.nodes.length,
      treeSizeGoal:  treeGoal.nodes.length,
      iteration:     itersRun,
      maxIter:       cfg.maxIter,
      found:         connection !== null,
      plannerType:   'rrt',
    });
    await new Promise(r => setTimeout(r, 0));

    if (connection === null) return fallback();

    let path = extractPath(treeStart, treeGoal, connection);
    path = await shortcut(path, collisionQuads, halfExtents, cfg, rng, shouldCancel);

    const { tightestIndex, minC } = computeTightest(path, collisionQuads, halfExtents);
    return { poses: path, fits: minC >= 0, tightestIndex };
  },

  formatProgress(data, container) {
    const {
      treeSizeStart = 0, treeSizeGoal = 0, iteration = 0,
      maxIter = DEFAULTS.maxIter, found = false,
    } = data;

    if (!container.querySelector('#tl-rrt-iter')) {
      container.innerHTML = `
        <span id="tl-rrt-trees" style="font-family:monospace;font-size:12px;color:#64ffda;">A:0 B:0</span>
        <span id="tl-rrt-iter"  style="font-family:monospace;font-size:12px;color:#aaa;">0 / ${maxIter.toLocaleString()}</span>
        <span id="tl-rrt-found" style="font-family:monospace;font-size:12px;color:#888;">searching…</span>
        <div style="flex:1;height:6px;background:#1a2a3a;border-radius:3px;overflow:hidden;">
          <div id="tl-rrt-bar" style="height:100%;width:0%;background:#64ffda;border-radius:3px;
            transition:width 0.4s;"></div>
        </div>`;
    }

    const t = container.querySelector('#tl-rrt-trees');
    const i = container.querySelector('#tl-rrt-iter');
    const f = container.querySelector('#tl-rrt-found');
    const b = container.querySelector('#tl-rrt-bar');
    if (t) t.textContent = `A:${treeSizeStart} B:${treeSizeGoal}`;
    if (i) i.textContent = `${iteration.toLocaleString()} / ${maxIter.toLocaleString()}`;
    if (f) {
      f.textContent  = found ? 'path found ✓' : 'searching…';
      f.style.color  = found ? '#22ff88' : '#888';
    }
    if (b) b.style.width = `${Math.min(100, (iteration / maxIter) * 100)}%`;
  },
};
