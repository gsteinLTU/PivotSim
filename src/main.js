import * as THREE from 'three';
import { createScene } from './viewer/scene.js';
import { createConfigPanel } from './ui/config-panel.js';
import { buildStairwell } from './geometry/stairwell.js';
import { buildQuadDebug } from './viewer/debug.js';
import { createBoxMesh, updateBoxMeshPose, computeOBB, getHalfExtents } from './geometry/box.js';
import { checkCollisions } from './solver/collision.js';
import { DEFAULTS, BOX_DEFAULTS, BOX_POSE_DEFAULTS } from './defaults.js';

const DEG = Math.PI / 180;

const viewport = document.getElementById('viewport');
const configContainer = document.getElementById('config-panel');
const readout = document.getElementById('clearance-readout');

const { scene, camera, renderer, controls } = createScene(viewport);

let currentStairwell = null;
let currentQuadDebug = null;
let currentBox = null;
let currentCollisionQuads = [];
let currentBoxDims = { ...BOX_DEFAULTS };
let currentBoxPose = { ...BOX_POSE_DEFAULTS };

// Convert pose from degrees (UI) to radians (math)
function poseRad(pose) {
  return {
    x: pose.x, y: pose.y, z: pose.z,
    yaw: pose.yaw * DEG,
    pitch: pose.pitch * DEG,
    roll: pose.roll * DEG,
  };
}

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

function rebuildStairwell(params) {
  if (currentStairwell) {
    scene.remove(currentStairwell);
    currentStairwell.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const prevQuadDebugVisible = currentQuadDebug ? currentQuadDebug.visible : false;
  if (currentQuadDebug) {
    scene.remove(currentQuadDebug);
    currentQuadDebug.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const { group, collisionQuads } = buildStairwell(params);
  currentCollisionQuads = collisionQuads;
  scene.add(group);
  currentStairwell = group;

  currentQuadDebug = buildQuadDebug(collisionQuads);
  currentQuadDebug.visible = prevQuadDebugVisible;
  scene.add(currentQuadDebug);

  // Auto-frame camera
  const box = new THREE.Box3().setFromObject(group);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const maxDim = Math.max(size.x, size.y, size.z);
  controls.target.copy(center);
  camera.position.set(
    center.x + maxDim * 1.2,
    center.y + maxDim * 0.8,
    center.z + maxDim * 1.2
  );
  controls.update();

  updateBoxCollision();
}

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
}

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

// Initial build
rebuildStairwell(panel.getParams());
rebuildBox();
readout.style.display = 'block';

// Animation loop
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

// Resize handler
window.addEventListener('resize', () => {
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
});
