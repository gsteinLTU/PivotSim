import { buildStairwell } from '../geometry/stairwell.js';
import { getHalfExtents } from '../geometry/box.js';
import { buildCenterline, buildContainmentOBBs, getSegmentBoundaries, getCorridorYaws } from './path.js';
import { checkCollisions } from './collision.js';
import { computeOBBFromPose } from './utils.js';

const DEFAULT_GOAL_YAW_OFFSETS = [0, Math.PI / 2, Math.PI, -Math.PI / 2];

export function buildPlannerContext(stairwellParams, boxDims, goalYawOffsets = DEFAULT_GOAL_YAW_OFFSETS) {
  const { collisionQuads } = buildStairwell(stairwellParams);
  const halfExtents        = getHalfExtents(boxDims);
  const centerline         = buildCenterline(stairwellParams);
  const containmentOBBs    = buildContainmentOBBs(centerline, stairwellParams);
  const boundaries         = getSegmentBoundaries(centerline);
  const { startYaw, endYaw } = getCorridorYaws(centerline);

  const { ceilingHeight, startHallCenter, endHallCenter } = centerline;
  const halfCeil = ceilingHeight / 2;
  const startPt  = startHallCenter;
  const endPt    = endHallCenter;

  const startPoseCandidate = { x: startPt[0], y: startPt[1] + halfCeil, z: startPt[2], yaw: startYaw, pitch: 0, roll: 0 };
  const startPose = checkCollisions(computeOBBFromPose(startPoseCandidate, halfExtents), collisionQuads).minClearance >= 0
    ? startPoseCandidate
    : { x: startPt[0], y: startPt[1] + halfCeil, z: startPt[2], yaw: 0, pitch: 0, roll: 0 };

  const endPosition = { x: endPt[0], y: endPt[1] + halfCeil, z: endPt[2] };
  const endPoses = goalYawOffsets
    .map(offset => ({ ...endPosition, yaw: endYaw + offset, pitch: 0, roll: 0 }))
    .filter(pose => checkCollisions(computeOBBFromPose(pose, halfExtents), collisionQuads).minClearance >= 0);

  const endPose = endPoses.length > 0
    ? endPoses[0]
    : { ...endPosition, yaw: endYaw, pitch: 0, roll: 0 };

  const quadsBySegment = {
    stair:         collisionQuads.filter(q => q.segment === 'stair'),
    'bottom-hall': collisionQuads.filter(q => q.segment === 'bottom-hall'),
    'top-hall':    collisionQuads.filter(q => q.segment === 'top-hall'),
  };

  return {
    collisionQuads, halfExtents, startPose, endPose, endPoses,
    containmentOBBs, centerline, quadsBySegment, boundaries,
  };
}
