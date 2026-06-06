const DEG = Math.PI / 180;

/**
 * Builds a centerline polyline through the stairwell.
 * Points run: bottom hallway far end → stair base → step tread centers
 *             → stair top → top hallway far end.
 * Returns { points: [[x,y,z], ...], totalLength: number }
 */
export function buildCenterline(params) {
  const {
    numSteps, risePerStep, runPerStep,
    bottomHallwayTurn, topHallwayTurn, hallwayLength,
  } = params;

  const totalRise = numSteps * risePerStep;
  const totalRun  = numSteps * runPerStep;
  const btRad = bottomHallwayTurn * DEG;
  const ttRad = topHallwayTurn * DEG;

  const points = [];

  // Bottom hallway far end (box enters here)
  points.push([
    Math.sin(btRad) * hallwayLength,
    0,
    -Math.cos(btRad) * hallwayLength,
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

  // Top hallway far end (box exits here)
  // Top hallway rotation: π - ttRad → local -Z in world = [-sin(π-ttRad), 0, -cos(π-ttRad)]
  //                                                      = [-sin(ttRad), 0, cos(ttRad)]
  points.push([
    -Math.sin(ttRad) * hallwayLength,
    totalRise,
    totalRun + Math.cos(ttRad) * hallwayLength,
  ]);

  // Arc length
  let totalLength = 0;
  for (let i = 1; i < points.length; i++) {
    const [x0, y0, z0] = points[i - 1];
    const [x1, y1, z1] = points[i];
    totalLength += Math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2);
  }

  return { points, totalLength };
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
