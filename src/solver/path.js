const DEG = Math.PI / 180;

/**
 * Builds a centerline polyline through the stairwell.
 * Points run: bottom hallway far end → stair base → step tread centers
 *             → stair top → top hallway far end.
 *
 * Far-end coordinates match the world positions produced by buildHallway,
 * including the stairHalfW / hallHalfW position offsets applied on turns.
 *
 * Returns { points: [[x,y,z], ...], totalLength: number, ceilingHeight: number }
 */
export function buildCenterline(params) {
  const {
    numSteps, risePerStep, runPerStep,
    bottomHallwayTurn, topHallwayTurn, hallwayLength,
    stairWidth, bottomHallwayWidth, topHallwayWidth, ceilingHeight,
  } = params;

  const totalRise  = numSteps * risePerStep;
  const totalRun   = numSteps * runPerStep;
  const btRad      = bottomHallwayTurn * DEG;
  const ttRad      = topHallwayTurn    * DEG;
  const stairHalfW = stairWidth / 2;
  const btHalfW    = bottomHallwayWidth / 2;
  const ttHalfW    = topHallwayWidth    / 2;
  const L          = hallwayLength;

  const points = [];

  // ── Bottom hallway far end ──────────────────────────────────────────────
  // buildHallway rotates by -btRad and (when turned) shifts by:
  //   x -= stairHalfW * sign,  z -= btHalfW
  // Local far end [0,0,-L] after rotation.y = -btRad becomes [L·sin(btRad), 0, -L·cos(btRad)].
  // Add the position offsets to get world coordinates.
  const btSign = bottomHallwayTurn > 0 ? 1 : -1;
  const btXOff = bottomHallwayTurn !== 0 ? -stairHalfW * btSign : 0;
  const btZOff = bottomHallwayTurn !== 0 ? -btHalfW              : 0;
  points.push([
    L * Math.sin(btRad) + btXOff,
    0,
    -L * Math.cos(btRad) + btZOff,
  ]);

  // Stair base junction
  points.push([0, 0, 0]);

  // One point per step tread center
  for (let i = 0; i < numSteps; i++) {
    points.push([
      0,
      i * risePerStep + risePerStep / 2,
      i * runPerStep  + runPerStep  / 2,
    ]);
  }

  // Stair top junction
  points.push([0, totalRise, totalRun]);

  // ── Top hallway far end ─────────────────────────────────────────────────
  // buildHallway rotates by π - ttRad and (when turned) shifts by:
  //   x += stairHalfW * sign,  z += ttHalfW
  // Local far end [0,0,-L] after rotation.y = π - ttRad becomes [-L·sin(ttRad), 0, L·cos(ttRad)].
  // Add position base [0, totalRise, totalRun] plus offsets.
  const ttSign = topHallwayTurn > 0 ? 1 : -1;
  const ttXOff = topHallwayTurn !== 0 ? stairHalfW * ttSign : 0;
  const ttZOff = topHallwayTurn !== 0 ? ttHalfW              : 0;
  points.push([
    -L * Math.sin(ttRad) + ttXOff,
    totalRise,
    totalRun + L * Math.cos(ttRad) + ttZOff,
  ]);

  // Arc length
  let totalLength = 0;
  for (let i = 1; i < points.length; i++) {
    const [x0, y0, z0] = points[i - 1];
    const [x1, y1, z1] = points[i];
    totalLength += Math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2);
  }

  return { points, totalLength, ceilingHeight };
}

/**
 * Returns the entry and exit endpoints of the centerline.
 * start = bottom hallway far end, end = top hallway far end.
 */
export function getEndpoints(centerline) {
  const { points } = centerline;
  return {
    start: points[0],
    end:   points[points.length - 1],
  };
}

/**
 * Builds three generous containment OBBs (one per zone) used to detect void-space escape.
 * A pose outside ALL three OBBs is penalized as "void."
 *
 * Zone 1 — stair flight: world-space AABB covering x ∈ [±stairWidth/2], full rise+ceiling, full run.
 * Zone 2 — bottom hallway: horizontal OBB aligned with hallway centerline direction.
 * Zone 3 — top hallway: horizontal OBB aligned with hallway centerline direction.
 *
 * Returns [stairOBB, bottomOBB, topOBB]  ({ center, axes, halfExtents } each)
 */
export function buildContainmentOBBs(centerline, params) {
  const { points, ceilingHeight } = centerline;
  const { stairWidth, bottomHallwayWidth, topHallwayWidth } = params;
  const M = 0.4;   // generous margin so valid poses are never penalized
  const n = points.length;
  const halfCeil = ceilingHeight / 2;

  const stairBase = points[1];    // world (0, 0, 0)
  const stairTop  = points[n - 2]; // world (0, totalRise, totalRun)

  // ── Zone 1: stair flight ─────────────────────────────────────────────────
  // Pure AABB — stairs always run along +Z/+Y with no X component.
  const stairRise = stairTop[1];
  const stairRun  = stairTop[2];
  const stairOBB  = {
    center:      [0, (stairRise + ceilingHeight) / 2, stairRun / 2],
    axes:        [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    halfExtents: [stairWidth / 2 + M, (stairRise + ceilingHeight) / 2 + M, stairRun / 2 + M],
  };

  // ── Build a horizontal corridor OBB ──────────────────────────────────────
  // ptA / ptB are both at y = floor level; the OBB is axis-aligned vertically.
  function hCorrOBB(ptA, ptB, halfWidth) {
    const dx = ptB[0] - ptA[0], dz = ptB[2] - ptA[2];
    const len = Math.sqrt(dx * dx + dz * dz);
    if (len < 1e-6) return null;
    const fwd  = [dx / len, 0, dz / len];
    const side = [-dz / len, 0, dx / len];
    const baseY = (ptA[1] + ptB[1]) / 2;
    return {
      center:      [(ptA[0] + ptB[0]) / 2, baseY + halfCeil, (ptA[2] + ptB[2]) / 2],
      axes:        [side, [0, 1, 0], fwd],
      halfExtents: [halfWidth + M, halfCeil + M, len / 2 + M],
    };
  }

  const bHW     = Math.max(bottomHallwayWidth, stairWidth) / 2;
  const bottomOBB = hCorrOBB(points[0], stairBase, bHW);

  const tHW   = Math.max(topHallwayWidth, stairWidth) / 2;
  const topOBB  = hCorrOBB(stairTop, points[n - 1], tHW);

  return [stairOBB, bottomOBB, topOBB].filter(Boolean);
}
