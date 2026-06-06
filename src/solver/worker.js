import { buildStairwell } from '../geometry/stairwell.js';
import { getHalfExtents } from '../geometry/box.js';
import { buildCenterline } from './path.js';
import { optimizeTrajectory } from './trajectory.js';

let cancelFlag = false;

self.onmessage = async ({ data }) => {
  if (data.type === 'cancel') {
    cancelFlag = true;
    return;
  }

  if (data.type === 'start') {
    cancelFlag = false;
    const { stairwellParams, boxDims, weights } = data;
    try {
      const { collisionQuads } = buildStairwell(stairwellParams);
      const halfExtents        = getHalfExtents(boxDims);
      const centerline         = buildCenterline(stairwellParams);

      const result = await optimizeTrajectory(
        collisionQuads,
        halfExtents,
        centerline,
        weights,
        (progress) => {
          self.postMessage({ type: 'progress', ...progress });
        },
        () => cancelFlag,
      );

      if (cancelFlag) {
        self.postMessage({ type: 'canceled', ...result });
      } else {
        self.postMessage({ type: 'done', ...result });
      }
    } catch (err) {
      self.postMessage({ type: 'error', message: err.message });
    }
  }
};
