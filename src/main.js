import * as THREE from 'three';
import { createScene } from './viewer/scene.js';
import { createConfigPanel } from './ui/config-panel.js';
import { buildStairwell } from './geometry/stairwell.js';
import { buildQuadDebug } from './viewer/debug.js';
import { createBoxMesh, updateBoxMeshPose, computeOBB, getHalfExtents } from './geometry/box.js';
import { checkCollisions } from './solver/collision.js';
import { DEFAULTS, BOX_DEFAULTS, BOX_POSE_DEFAULTS } from './defaults.js';
import { createTimeline } from './ui/timeline.js';
import { lerpPose } from './solver/trajectory.js';

const DEG = Math.PI / 180;
const MAX_GHOST = 20;

const viewport         = document.getElementById('viewport');
const configContainer  = document.getElementById('config-panel');
const readout          = document.getElementById('clearance-readout');
const timelineContainer = document.getElementById('timeline');

const { scene, camera, renderer, controls } = createScene(viewport);

// ── Stairwell state ────────────────────────────────────────────────────────
let currentStairwell    = null;
let currentQuadDebug    = null;
let currentBox          = null;
let currentCollisionQuads = [];
let currentBoxDims      = { ...BOX_DEFAULTS };
let currentBoxPose      = { ...BOX_POSE_DEFAULTS };

// ── Trajectory state ───────────────────────────────────────────────────────
let currentTrajectory   = null;   // TrajectoryResult (poses in radians) | null
let currentWorker       = null;
let isPlaying           = false;
let playSpeed           = 1.0;
let playheadSeconds     = 0;
const ghostMeshes       = [];     // fixed pool, created at solve start

function poseRad(pose) {
  return {
    x: pose.x, y: pose.y, z: pose.z,
    yaw: pose.yaw * DEG, pitch: pose.pitch * DEG, roll: pose.roll * DEG,
  };
}

// ── Collision readout ──────────────────────────────────────────────────────
function updateBoxCollision() {
  if (!currentBox || currentCollisionQuads.length === 0) return;
  const obb = computeOBB(poseRad(currentBoxPose), getHalfExtents(currentBoxDims));
  const { collides, minClearance } = checkCollisions(obb, currentCollisionQuads);

  if (collides) {
    currentBox.material.color.setHex(0xff2222);
    readout.textContent = 'COLLISION';
    readout.style.color = '#ff4444';
  } else if (minClearance < 0.05) {
    currentBox.material.color.setHex(0xffaa00);
    readout.textContent = `Tight: ${(minClearance * 100).toFixed(1)} cm`;
    readout.style.color = '#ffaa00';
  } else {
    currentBox.material.color.setHex(0x22ff88);
    readout.textContent = `Clear: ${(minClearance * 100).toFixed(0)} cm`;
    readout.style.color = '#22ff88';
  }
}

// ── Stairwell rebuild ──────────────────────────────────────────────────────
function rebuildStairwell(params) {
  if (currentStairwell) {
    scene.remove(currentStairwell);
    currentStairwell.traverse((c) => {
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    });
  }
  const prevVisible = currentQuadDebug ? currentQuadDebug.visible : false;
  if (currentQuadDebug) {
    scene.remove(currentQuadDebug);
    currentQuadDebug.traverse((c) => {
      if (c.geometry) c.geometry.dispose();
      if (c.material) c.material.dispose();
    });
  }

  const { group, collisionQuads } = buildStairwell(params);
  currentCollisionQuads = collisionQuads;
  scene.add(group);
  currentStairwell = group;

  currentQuadDebug = buildQuadDebug(collisionQuads);
  currentQuadDebug.visible = prevVisible;
  scene.add(currentQuadDebug);

  const box = new THREE.Box3().setFromObject(group);
  const center = box.getCenter(new THREE.Vector3());
  const size   = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z);
  controls.target.copy(center);
  camera.position.set(
    center.x + maxDim * 1.2, center.y + maxDim * 0.8, center.z + maxDim * 1.2,
  );
  controls.update();
  updateBoxCollision();

  // Stale result when geometry changes — terminate any running worker immediately
  if (currentWorker || currentTrajectory) {
    currentWorker?.terminate();
    currentWorker = null;
    panel.unlock();
    currentTrajectory = null;
    isPlaying = false;
    clearGhostTrail();
    timeline.setState('idle');
  }
}

// ── Box rebuild ────────────────────────────────────────────────────────────
function rebuildBox() {
  if (currentBox) {
    scene.remove(currentBox);
    currentBox.geometry.dispose();
    currentBox.material.dispose();
  }
  currentBox = createBoxMesh(currentBoxDims);
  updateBoxMeshPose(currentBox, poseRad(currentBoxPose));
  scene.add(currentBox);
  updateBoxCollision();

  // Stale result when box dims change
  if (currentWorker || currentTrajectory) {
    cancelSolve();
    currentTrajectory = null;
    isPlaying = false;
    clearGhostTrail();
    timeline.setState('idle');
  }
}

// ── Ghost trail ────────────────────────────────────────────────────────────
function createGhostPool() {
  ghostMeshes.forEach((m) => { scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
  ghostMeshes.length = 0;
  for (let i = 0; i < MAX_GHOST; i++) {
    const m = createBoxMesh(currentBoxDims);
    m.material = m.material.clone();
    m.material.opacity = 0.12;
    m.material.color.setHex(0x6688ff);
    m.visible = false;
    scene.add(m);
    ghostMeshes.push(m);
  }
}

function clearGhostTrail() {
  ghostMeshes.forEach((m) => { m.visible = false; });
}

function renderGhostTrail(poses) {
  if (!poses || poses.length === 0) { clearGhostTrail(); return; }
  const n    = Math.min(poses.length, MAX_GHOST);
  const step = poses.length > 1 ? (poses.length - 1) / (n - 1) : 0;
  for (let g = 0; g < MAX_GHOST; g++) {
    if (g < n) {
      const idx = Math.min(poses.length - 1, Math.round(g * step));
      updateBoxMeshPose(ghostMeshes[g], poses[idx]);   // poses already in radians
      ghostMeshes[g].visible = true;
    } else {
      ghostMeshes[g].visible = false;
    }
  }
}

// ── Solve flow ─────────────────────────────────────────────────────────────
function startSolve() {
  panel.lock();
  timeline.setState('solving');
  isPlaying = false;
  playheadSeconds = 0;
  clearGhostTrail();
  createGhostPool();

  currentWorker = new Worker(new URL('./solver/worker.js', import.meta.url), { type: 'module' });
  currentWorker.onmessage = ({ data }) => {
    if (data.type === 'progress') {
      timeline.updateProgress(data);
      renderGhostTrail(data.poses);
    } else {
      // 'done', 'canceled', or 'error'
      currentWorker = null;
      panel.unlock();
      currentTrajectory = (data.poses?.length >= 2) ? data : null;
      timeline.setResult({
        fits:          data.fits,
        tightestIndex: data.tightestIndex,
        poses:         data.poses,
        segmentTimes:  data.segmentTimes,
        totalTime:     data.totalTime,
      });
    }
  };
  currentWorker.postMessage({
    type: 'start',
    stairwellParams: panel.getParams(),
    boxDims: currentBoxDims,
  });
}

function cancelSolve() {
  if (currentWorker) {
    currentWorker.postMessage({ type: 'cancel' });
  }
}

// ── Playback ───────────────────────────────────────────────────────────────
function poseAtTime(trajectory, seconds) {
  let elapsed = 0;
  const { poses, segmentTimes } = trajectory;
  for (let i = 0; i < segmentTimes.length; i++) {
    const dt = segmentTimes[i];
    if (elapsed + dt >= seconds || i === segmentTimes.length - 1) {
      const t = dt > 0 ? Math.min(1, (seconds - elapsed) / dt) : 0;
      return lerpPose(poses[i], poses[i + 1], t);
    }
    elapsed += dt;
  }
  return { ...poses[poses.length - 1] };
}

function onPlayheadChange(seconds) {
  playheadSeconds = Math.max(0, Math.min(seconds, currentTrajectory?.totalTime ?? 0));
  if (!currentTrajectory) return;
  const pose = poseAtTime(currentTrajectory, playheadSeconds);
  updateBoxMeshPose(currentBox, pose);   // pose from trajectory is already in radians
  updateBoxCollision();
  timeline.updatePlayhead(playheadSeconds);
}

// ── Config panel ───────────────────────────────────────────────────────────
const panel = createConfigPanel(configContainer, { ...DEFAULTS }, (params) => {
  rebuildStairwell(params);
});

panel.onCeilingToggle((visible) => {
  if (!currentStairwell) return;
  currentStairwell.traverse((child) => {
    if (child.userData.isSurface) child.visible = visible;
  });
});

panel.onQuadDebugToggle((visible) => {
  if (currentQuadDebug) currentQuadDebug.visible = visible;
});

panel.onBoxDimsChange((dims) => {
  currentBoxDims = dims;
  rebuildBox();
});

panel.onBoxPoseChange((pose) => {
  currentBoxPose = pose;
  if (currentBox) {
    updateBoxMeshPose(currentBox, poseRad(pose));
    updateBoxCollision();
  }
});

// ── Timeline ───────────────────────────────────────────────────────────────
const timeline = createTimeline(timelineContainer, {
  onSolve:         startSolve,
  onCancel:        cancelSolve,
  onPlayheadChange: (seconds) => { isPlaying = false; timeline.setPlayState(false); onPlayheadChange(seconds); },
  onPlayToggle:    (playing)  => { isPlaying = playing; },
  onSpeedChange:   (speed)    => { playSpeed = speed; },
  onReset:         ()         => { currentTrajectory = null; isPlaying = false; clearGhostTrail(); },
});

// ── Initial build ──────────────────────────────────────────────────────────
rebuildStairwell(panel.getParams());
rebuildBox();
readout.style.display = 'block';

// ── Animation loop ─────────────────────────────────────────────────────────
let lastTimestamp = 0;
function animate(timestamp) {
  requestAnimationFrame(animate);
  controls.update();

  const dt = lastTimestamp > 0 ? (timestamp - lastTimestamp) / 1000 : 0;
  lastTimestamp = timestamp;

  if (currentTrajectory && isPlaying) {
    playheadSeconds += dt * playSpeed;
    if (playheadSeconds >= currentTrajectory.totalTime) {
      playheadSeconds = currentTrajectory.totalTime;
      isPlaying = false;
      timeline.setPlayState(false);
    }
    onPlayheadChange(playheadSeconds);
  }

  renderer.render(scene, camera);
}
animate(0);

// ── Resize ─────────────────────────────────────────────────────────────────
window.addEventListener('resize', () => {
  const w = viewport.clientWidth, h = viewport.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});
