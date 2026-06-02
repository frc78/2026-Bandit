// Field dimensions derived from FieldConstants.MIDPOINT = (325.61", 158.84")
export const FIELD_WIDTH_M  = 325.61 * 2 * 0.0254; // ~16.541 m
export const FIELD_HEIGHT_M = 158.84 * 2 * 0.0254; // ~8.069 m

// Robot dimensions (frame + bumpers on each side)
// Frame: 28" x 26", bumpers: ~3" each side
export const ROBOT_LENGTH_M = (28 + 6) * 0.0254; // 34" front-to-back = 0.8636 m
export const ROBOT_WIDTH_M  = (26 + 6) * 0.0254; // 32" side-to-side  = 0.8128 m

// Intake extension past front bumper face
export const INTAKE_EXT_M = 9 * 0.0254; // 9" = 0.2286 m

// Drivetrain (from TunerConstants.kSpeedAt12Volts)
export const MAX_SPEED_MS = 3.79;
// MaxAutoRotationSpeed = kSpeedAt12Volts / hypot(BackLeft.LocationX, BackLeft.LocationY)
// BackLeft: X = -10.375", Y = 11.375" (from TunerConstants.java)
export const DEFAULT_MAX_ROTATION = MAX_SPEED_MS / Math.hypot(10.375 * 0.0254, 11.375 * 0.0254); // ~9.69 rad/s
