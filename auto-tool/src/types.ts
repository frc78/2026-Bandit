export type IntakeState = 'Unchanged' | 'Retracted' | 'Intaking' | 'Deployed' | 'Jostle';
export type FlywheelState = 'Unchanged' | 'Still' | 'Firing';

export interface StartPose {
  x: number;           // meters, WPILib blue alliance coords
  y: number;           // meters
  rotationDeg: number; // degrees, WPILib CCW from +x
  intakeState: IntakeState;
  flywheelState: FlywheelState;
}

export type FinishCondition = 'AND' | 'OR';

export interface Leg {
  id: string;
  name: string;        // PascalCase; used as Step enum name and camelCase var name
  x: number;           // meters
  y: number;           // meters
  rotationDeg: number; // degrees
  thresholdM: number;  // meters; distance value used when finishThreshold is true
  maxSpeed: number | null;    // null = use robot default (kSpeedAt12Volts)
  maxRotation: number | null; // null = use robot default (MaxAutoRotationSpeed)
  alwaysMaxSpeed: boolean;
  minSpeed: number;    // m/s
  intakeFirst: boolean;
  // Finish conditions (all off = just drive, never advance)
  finishThreshold: boolean;      // advance when within thresholdM of target
  finishLegTimer: number | null; // advance after N seconds from start of this leg
  finishAutoTimer: number | null;// advance after N seconds from auto start
  finishCondition: FinishCondition; // how to combine when ≥2 conditions active
  intakeState: IntakeState;    // set when finish condition is met
  flywheelState: FlywheelState;
}

export interface AutoConfig {
  name: string; // Kotlin object name (PascalCase)
  startPose: StartPose;
  legs: Leg[];
}
