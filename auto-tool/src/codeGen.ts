import type { AutoConfig, Leg, IntakeState, FlywheelState } from './types';

function toVarName(s: string): string {
  return s.charAt(0).toLowerCase() + s.slice(1);
}

function toEnumName(s: string): string {
  return s.charAt(0).toUpperCase() + s.slice(1);
}

function formatMeters(m: number): string {
  const rounded = Math.round(m * 1000) / 1000;
  return `${rounded}.meters`;
}

function fmtDouble(v: number): string {
  return Number.isInteger(v) ? `${v}.0` : String(v);
}

function intakeCode(state: IntakeState): string | null {
  if (state === 'Unchanged') return null;
  return `Intake.currentState = IntakeState.${state}`;
}

function flywheelCode(state: FlywheelState): string | null {
  if (state === 'Unchanged') return null;
  return `Shooter.flywheelState = Shooter.FlywheelState.${state}`;
}

function buildDriveCall(leg: Leg, includeThreshold: boolean): string {
  const vn = toVarName(leg.name);
  const parts: string[] = [vn];

  if (includeThreshold && leg.thresholdM > 0) {
    parts.push(formatMeters(leg.thresholdM));
  }
  if (leg.alwaysMaxSpeed) parts.push('alwaysMaxSpeed = true');
  if (leg.maxSpeed !== null) parts.push(`maxSpeed = ${fmtDouble(leg.maxSpeed)}`);
  if (leg.maxRotation !== null) parts.push(`maxRotation = ${fmtDouble(leg.maxRotation)}`);
  if (leg.minSpeed > 0) parts.push(`minSpeed = ${fmtDouble(leg.minSpeed)}`);
  if (leg.intakeFirst) parts.push('intakeFirst = true');

  return `SwerveDrivetrain.driveToPose(${parts.join(', ')})`;
}

function fmtSecs(v: number): string {
  return Number.isInteger(v) ? `${v}.0` : String(v);
}

function legTimerName(leg: Leg): string {
  return `${toVarName(leg.name)}Timer`;
}

export function generateKotlin(auto: AutoConfig): string {
  const { name, startPose, legs } = auto;

  const names = legs.map(l => l.name);
  const dupes = [...new Set(names.filter((n, i) => names.indexOf(n) !== i))];
  if (dupes.length > 0) {
    throw new Error(`Duplicate leg name${dupes.length > 1 ? 's' : ''}: ${dupes.join(', ')}`);
  }

  const needsIntake = [startPose as { intakeState: IntakeState }, ...legs].some(
    l => l.intakeState !== 'Unchanged'
  );
  const needsFlywheel = [startPose as { flywheelState: FlywheelState }, ...legs].some(
    l => l.flywheelState !== 'Unchanged'
  );
  const legsWithLegTimer = legs.filter(l => l.finishLegTimer !== null);
  const needsAutoTimer = legs.some(l => l.finishAutoTimer !== null);
  const needsTimer = legsWithLegTimer.length > 0 || needsAutoTimer;

  const L: string[] = [];
  const push = (...lines: string[]) => L.push(...lines);

  push(
    'package frc.robot.auto',
    '',
    'import com.ctre.phoenix6.swerve.SwerveRequest',
    'import edu.wpi.first.math.geometry.Rotation2d',
    'import edu.wpi.first.wpilibj.RobotBase',
  );
  if (needsTimer) push('import edu.wpi.first.wpilibj.Timer');
  push(
    'import frc.robot.FieldLocation',
    'import frc.robot.lib.degrees',
    'import frc.robot.lib.meters',
  );
  if (needsIntake) {
    push(
      'import frc.robot.subsystems.Intake',
      'import frc.robot.subsystems.Intake.IntakeState',
    );
  }
  if (needsFlywheel) {
    push('import frc.robot.subsystems.Shooter');
  }
  push(
    'import frc.robot.subsystems.SwerveDrivetrain',
    'import org.littletonrobotics.junction.Logger',
    '',
    `object ${name} : Auto {`,
    '    enum class Step {',
  );

  // Build enum entries
  const allSteps = ['AutoStart', ...legs.map(l => toEnumName(l.name)), 'Finished'];
  allSteps.forEach((s, i) => {
    const isLast = i === allSteps.length - 1;
    push(`        ${s}${isLast ? ';' : ','}`);
  });

  push(
    '',
    '        operator fun inc(): Step {',
    '            return entries.getOrNull(ordinal + 1) ?: this',
    '        }',
    '    }',
    '',
    '    var step = Step.AutoStart',
    '',
  );

  // Field location declarations
  push(
    `    private val autoStartPoint by FieldLocation(${startPose.x}, ${startPose.y}, Rotation2d((${startPose.rotationDeg}).degrees))`,
  );
  legs.forEach(leg => {
    push(
      `    private val ${toVarName(leg.name)} by FieldLocation(${leg.x}, ${leg.y}, Rotation2d((${leg.rotationDeg}).degrees))`,
    );
  });

  // Timer field declarations
  if (needsAutoTimer) push('    private val autoTimer = Timer()');
  legsWithLegTimer.forEach(leg => {
    push(`    private val ${legTimerName(leg)} = Timer()`);
  });

  push(
    '',
    '    override fun init() {',
    '        step = Step.AutoStart',
  );
  // Stop leg timers so they restart cleanly on re-run
  legsWithLegTimer.forEach(leg => {
    push(`        ${legTimerName(leg)}.stop()`);
    push(`        ${legTimerName(leg)}.reset()`);
  });
  push('    }', '');

  push(
    '    override fun run() {',
    '        Logger.recordOutput("auto/step", step.name)',
    '        when (step) {',
    '            Step.AutoStart -> {',
    '                if (RobotBase.isSimulation()) {',
    '                    SwerveDrivetrain.resetPose(autoStartPoint)',
    '                }',
  );
  const si = intakeCode(startPose.intakeState);
  const sf = flywheelCode(startPose.flywheelState);
  if (si) push(`                ${si}`);
  if (sf) push(`                ${sf}`);
  if (needsAutoTimer) push('                autoTimer.restart()');
  push('                step++', '            }');

  // Each leg
  legs.forEach(leg => {
    const ai = intakeCode(leg.intakeState);
    const af = flywheelCode(leg.flywheelState);
    const stepName = toEnumName(leg.name);
    const hasLegTimer = leg.finishLegTimer !== null;
    const hasAutoTimer = leg.finishAutoTimer !== null;

    // Collect active finish conditions
    const condParts: string[] = [];
    if (leg.finishThreshold) condParts.push('atPose');
    if (hasLegTimer) condParts.push(`${legTimerName(leg)}.hasElapsed(${fmtSecs(leg.finishLegTimer!)})`);
    if (hasAutoTimer) condParts.push(`autoTimer.hasElapsed(${fmtSecs(leg.finishAutoTimer!)})`);

    const op = leg.finishCondition === 'AND' ? ' && ' : ' || ';
    const hasAnyFinish = condParts.length > 0;
    // When threshold is involved alongside other conditions we need a val to avoid calling twice
    const needsAtPose = leg.finishThreshold && condParts.length > 1;
    // Simple single-condition threshold: embed directly in if
    const thresholdOnly = leg.finishThreshold && condParts.length === 1;

    push(`            Step.${stepName} -> {`);

    // Start leg timer if needed (before drive call so timing is accurate)
    if (hasLegTimer) {
      push(`                if (!${legTimerName(leg)}.isRunning) ${legTimerName(leg)}.restart()`);
    }

    if (!hasAnyFinish) {
      // No finish condition — just drive
      push(`                ${buildDriveCall(leg, false)}`);
    } else if (thresholdOnly) {
      // Single condition, threshold only — simple if pattern
      push(`                if (${buildDriveCall(leg, true)}) {`);
      if (ai) push(`                    ${ai}`);
      if (af) push(`                    ${af}`);
      push('                    step++', '                }');
    } else {
      // Multi-condition or timer-only
      if (needsAtPose) {
        push(`                val atPose = ${buildDriveCall(leg, true)}`);
      } else {
        push(`                ${buildDriveCall(leg, false)}`);
      }
      const combined = condParts.join(op);
      push(`                if (${combined}) {`);
      if (ai) push(`                    ${ai}`);
      if (af) push(`                    ${af}`);
      push('                    step++', '                }');
    }

    push('            }');
  });

  push(
    '            Step.Finished -> {',
    '                SwerveDrivetrain.setControl(SwerveRequest.SwerveDriveBrake())',
    '            }',
    '        }',
    '    }',
    '}',
    '',
  );

  return L.join('\n');
}
