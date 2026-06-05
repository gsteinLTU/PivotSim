import * as THREE from 'three';
import { createScene } from './viewer/scene.js';
import { createConfigPanel } from './ui/config-panel.js';
import { buildStairwell } from './geometry/stairwell.js';
import { DEFAULTS } from './defaults.js';

const viewport = document.getElementById('viewport');
const configContainer = document.getElementById('config-panel');

const { scene, camera, renderer, controls } = createScene(viewport);

let currentStairwell = null;

function rebuildStairwell(params) {
  if (currentStairwell) {
    scene.remove(currentStairwell);
    currentStairwell.traverse((child) => {
      if (child.geometry) child.geometry.dispose();
      if (child.material) child.material.dispose();
    });
  }

  const { group } = buildStairwell(params);
  scene.add(group);
  currentStairwell = group;

  // Auto-frame the camera on the stairwell
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
}

const panel = createConfigPanel(configContainer, { ...DEFAULTS }, (params) => {
  rebuildStairwell(params);
});

panel.onCeilingToggle((visible) => {
  if (!currentStairwell) return;
  currentStairwell.traverse((child) => {
    if (child.userData.isSurface) {
      child.visible = visible;
    }
  });
});

// Initial build
rebuildStairwell(panel.getParams());

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
