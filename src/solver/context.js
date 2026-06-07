import { buildStairwell } from '../geometry/stairwell.js';
import { getHalfExtents } from '../geometry/box.js';
import { buildCenterline, buildContainmentOBBs, getSegmentBoundaries } from './path.js';

export function buildPlannerContext(stairwellParams, boxDims) {
  const { collisionQuads } = buildStairwell(stairwellParams);
  const halfExtents        = getHalfExtents(boxDims);
  const centerline         = buildCenterline(stairwellParams);
  const containmentOBBs    = buildContainmentOBBs(centerline, stairwellParams);
  const boundaries         = getSegmentBoundaries(centerline);

  const { points, ceilingHeight } = centerline;
  const halfCeil = ceilingHeight / 2;
  function midpoint(a, b) {
    return [(a[0]+b[0])/2, (a[1]+b[1])/2, (a[2]+b[2])/2];
  }
  const startPt = midpoint(points[0], points[1]);
  const endPt   = midpoint(points[points.length - 1], points[points.length - 2]);

  const startPose = { x: startPt[0], y: startPt[1] + halfCeil, z: startPt[2], yaw: 0, pitch: 0, roll: 0 };
  const endPose   = { x: endPt[0],   y: endPt[1]   + halfCeil, z: endPt[2],   yaw: 0, pitch: 0, roll: 0 };

  const quadsBySegment = {
    stair:         collisionQuads.filter(q => q.segment === 'stair'),
    'bottom-hall': collisionQuads.filter(q => q.segment === 'bottom-hall'),
    'top-hall':    collisionQuads.filter(q => q.segment === 'top-hall'),
  };

  const stairZone = {
    zMin: 0,
    zMax: boundaries.topTransitionPt[2],
    yMax: boundaries.topTransitionPt[1],
  };

  return {
    collisionQuads, halfExtents, startPose, endPose, containmentOBBs, centerline,
    quadsBySegment, boundaries, stairZone,
  };
}
