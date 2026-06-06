export const DEFAULTS = {
  stairWidth: 1.0,
  numSteps: 12,
  risePerStep: 0.19,
  runPerStep: 0.25,
  bottomHallwayWidth: 1.2,
  bottomHallwayTurn: 90,     // degrees: 0, 90, or -90
  topHallwayWidth: 1.2,
  topHallwayTurn: -90,        // degrees: 0, 90, or -90
  ceilingHeight: 2.4,
  slopedCeiling: true,
  hallwayLength: 3.0,       // default visualization length
};

export const BOX_DEFAULTS = {
  length: 0.5,   // m, longest dimension (runs along stairwell Z axis at rest)
  width: 0.8,    // m, horizontal cross-section
  height: 1.2,   // m, vertical dimension
};

export const BOX_POSE_DEFAULTS = {
  x: 0.1,
  y: 0.65,     // half of default height — box sits on floor
  z: -0.8,    // 1m into bottom hallway
  yaw: 0,     // degrees, rotation around Y (vertical) axis
  pitch: 0,   // degrees, rotation around X axis
  roll: 0,    // degrees, rotation around Z axis
};
