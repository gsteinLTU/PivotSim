export const MAX_LINEAR_SPEED  = 0.5;
export const MAX_ANGULAR_SPEED = 0.5;

function angleDiff(from, to) {
  let d = (to - from) % (2 * Math.PI);
  if (d >  Math.PI) d -= 2 * Math.PI;
  if (d < -Math.PI) d += 2 * Math.PI;
  return d;
}

export function euclideanDelta(a, b) {
  const dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  return Math.sqrt(dx*dx + dy*dy + dz*dz);
}

export function angularDelta(a, b) {
  return Math.abs(angleDiff(a.yaw,   b.yaw))
       + Math.abs(angleDiff(a.pitch, b.pitch))
       + Math.abs(angleDiff(a.roll,  b.roll));
}

export function segmentDuration(a, b) {
  return Math.max(
    euclideanDelta(a, b) / MAX_LINEAR_SPEED,
    angularDelta(a, b)   / MAX_ANGULAR_SPEED,
  );
}

export function lerpPose(a, b, t) {
  return {
    x:     a.x     + (b.x     - a.x)     * t,
    y:     a.y     + (b.y     - a.y)     * t,
    z:     a.z     + (b.z     - a.z)     * t,
    yaw:   a.yaw   + angleDiff(a.yaw,   b.yaw)   * t,
    pitch: a.pitch + angleDiff(a.pitch, b.pitch) * t,
    roll:  a.roll  + angleDiff(a.roll,  b.roll)  * t,
  };
}

export function applyRotationPropagation(poses, startIdx, endIdx, dof, delta) {
  return poses.map((p, j) =>
    j >= startIdx && j < endIdx ? { ...p, [dof]: p[dof] + delta } : { ...p }
  );
}
