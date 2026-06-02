import { useState, useEffect } from 'react';
import type { Leg, IntakeState, FlywheelState, FinishCondition } from '../types';
import { MAX_SPEED_MS, DEFAULT_MAX_ROTATION } from '../constants';

interface Props {
  leg: Leg;
  index: number;
  prevPose: { x: number; y: number };
  onChange: (updates: Partial<Leg>) => void;
}

function intakeFirstAngle(leg: Leg, prevPose: { x: number; y: number }): number {
  return Math.atan2(leg.y - prevPose.y, leg.x - prevPose.x) * (180 / Math.PI);
}

const INTAKE_STATES: IntakeState[] = ['Unchanged', 'Retracted', 'Intaking', 'Deployed', 'Jostle'];
const FLYWHEEL_STATES: FlywheelState[] = ['Unchanged', 'Still', 'Firing'];

function NumField({
  label,
  value,
  step = 0.01,
  onChange,
  unit,
  disabled,
}: {
  label: string;
  value: number;
  step?: number;
  onChange: (v: number) => void;
  unit?: string;
  disabled?: boolean;
}) {
  const [text, setText] = useState(String(value));

  useEffect(() => {
    if (parseFloat(text) !== value) setText(String(value));
  }, [value]);

  const invalid = text !== '' && isNaN(parseFloat(text));

  return (
    <label className="field-row">
      <span className="field-label">{label}</span>
      <div className="field-input-wrap">
        <input
          type="number"
          step={step}
          value={text}
          disabled={disabled}
          className={invalid ? 'input-error' : undefined}
          onChange={e => {
            setText(e.target.value);
            const v = parseFloat(e.target.value);
            if (!isNaN(v)) onChange(v);
          }}
          onBlur={() => {
            if (isNaN(parseFloat(text))) setText(String(value));
          }}
        />
        {unit && <span className="field-unit">{unit}</span>}
      </div>
    </label>
  );
}

function useNumInput(value: number, onChange: (v: number) => void) {
  const [text, setText] = useState(String(value));

  useEffect(() => {
    if (parseFloat(text) !== value) setText(String(value));
  }, [value]);

  const invalid = text !== '' && isNaN(parseFloat(text));

  return {
    value: text,
    className: invalid ? 'input-error' : undefined,
    onChange: (e: React.ChangeEvent<HTMLInputElement>) => {
      setText(e.target.value);
      const v = parseFloat(e.target.value);
      if (!isNaN(v)) onChange(v);
    },
    onBlur: () => {
      if (isNaN(parseFloat(text))) setText(String(value));
    },
  };
}

export default function LegEditor({ leg, index: _index, prevPose, onChange }: Props) {
  const condCount = [leg.finishThreshold, leg.finishLegTimer !== null, leg.finishAutoTimer !== null].filter(Boolean).length;

  const rotationVal = leg.intakeFirst ? Math.round(intakeFirstAngle(leg, prevPose) * 10) / 10 : leg.rotationDeg;
  const rotationInput = useNumInput(rotationVal, v => onChange({ rotationDeg: v }));
  const maxSpeedInput = useNumInput(leg.maxSpeed ?? MAX_SPEED_MS, v => onChange({ maxSpeed: v }));
  const maxRotationInput = useNumInput(
    Math.round((leg.maxRotation ?? DEFAULT_MAX_ROTATION) * (180 / Math.PI)),
    v => onChange({ maxRotation: v * (Math.PI / 180) }),
  );
  const thresholdInput = useNumInput(leg.thresholdM, v => onChange({ thresholdM: v }));
  const legTimerInput = useNumInput(leg.finishLegTimer ?? 3.0, v => onChange({ finishLegTimer: v }));
  const autoTimerInput = useNumInput(leg.finishAutoTimer ?? 5.0, v => onChange({ finishAutoTimer: v }));

  return (
    <>
      <section>
          <div className="section-title">Identity</div>
          <label className="field-row">
            <span className="field-label">Name</span>
            <input
              type="text"
              value={leg.name}
              onChange={e => onChange({ name: e.target.value })}
              placeholder="PascalCase"
            />
          </label>
        </section>

        <section>
          <div className="section-title">Target Pose</div>
          <NumField label="X" value={leg.x} unit="m" onChange={v => onChange({ x: v })} />
          <NumField label="Y" value={leg.y} unit="m" onChange={v => onChange({ y: v })} />
          <label className="field-row">
            <span className="field-label">Rotation</span>
            <div className="field-input-wrap">
              <input
                type="number"
                step={1}
                disabled={leg.intakeFirst}
                {...rotationInput}
              />
              <span className="field-unit">°</span>
            </div>
          </label>
          <div className="field-row checkbox-row">
            <span className="field-label">Intake First</span>
            <label>
              <input
                type="checkbox"
                checked={leg.intakeFirst}
                onChange={e => {
                  const checked = e.target.checked;
                  onChange({
                    intakeFirst: checked,
                    ...(checked ? { rotationDeg: intakeFirstAngle(leg, prevPose) } : {}),
                  });
                }}
              />
            </label>
          </div>
          {leg.intakeFirst && (
            <div className="field-hint">Facing intake toward target (auto-computed)</div>
          )}
        </section>

        <section>
          <div className="section-title">Drive Parameters</div>

          <div className="field-block">
            <div className="field-row">
              <span className="field-label">Max Speed</span>
              <label className="default-toggle">
                <input
                  type="checkbox"
                  checked={leg.maxSpeed === null}
                  onChange={e => onChange({ maxSpeed: e.target.checked ? null : MAX_SPEED_MS })}
                />
                Robot max
              </label>
            </div>
            <div className="field-row field-row-indented">
              <div className="field-input-wrap">
                <input type="number" step={0.1} disabled={leg.maxSpeed === null} {...maxSpeedInput} />
                <span className="field-unit">m/s</span>
              </div>
            </div>
          </div>

          <div className="field-row checkbox-row">
            <span className="field-label">Always Max Speed</span>
            <label>
              <input
                type="checkbox"
                checked={leg.alwaysMaxSpeed}
                onChange={e => onChange({ alwaysMaxSpeed: e.target.checked })}
              />
            </label>
          </div>

          <NumField
            label="Min Speed"
            value={leg.minSpeed}
            step={0.1}
            unit="m/s"
            onChange={v => onChange({ minSpeed: v })}
          />

          <div className="field-block">
            <div className="field-row">
              <span className="field-label">Max Rotation</span>
              <label className="default-toggle">
                <input
                  type="checkbox"
                  checked={leg.maxRotation === null}
                  onChange={e => onChange({ maxRotation: e.target.checked ? null : DEFAULT_MAX_ROTATION })}
                />
                Robot max
              </label>
            </div>
            <div className="field-row field-row-indented">
              <div className="field-input-wrap">
                <input type="number" step={5} disabled={leg.maxRotation === null} {...maxRotationInput} />
                <span className="field-unit">°/s</span>
              </div>
            </div>
          </div>
        </section>

        <section>
          <div className="section-title">Finish Parameters</div>

          <div className="field-row finish-row">
            <input type="checkbox" checked={leg.finishThreshold}
              onChange={e => onChange({ finishThreshold: e.target.checked })} />
            <span className="field-label">Threshold</span>
            <div className="field-input-wrap">
              <input type="number" step={0.1} disabled={!leg.finishThreshold} {...thresholdInput} />
              <span className="field-unit">m</span>
            </div>
          </div>

          <div className="field-row finish-row">
            <input type="checkbox" checked={leg.finishLegTimer !== null}
              onChange={e => onChange({ finishLegTimer: e.target.checked ? 3.0 : null })} />
            <span className="field-label">Leg timer</span>
            <div className="field-input-wrap">
              <input type="number" step={0.5} disabled={leg.finishLegTimer === null} {...legTimerInput} />
              <span className="field-unit">s</span>
            </div>
          </div>
          <div className="field-row finish-row">
            <input type="checkbox" checked={leg.finishAutoTimer !== null}
              onChange={e => onChange({ finishAutoTimer: e.target.checked ? 5.0 : null })} />
            <span className="field-label">Auto timer</span>
            <div className="field-input-wrap">
              <input type="number" step={0.5} disabled={leg.finishAutoTimer === null} {...autoTimerInput} />
              <span className="field-unit">s</span>
            </div>
          </div>

          {condCount >= 2 && (
            <div className="field-row" style={{ marginTop: 4 }}>
              <span className="field-label">Combine</span>
              <div className="finish-condition-toggle">
                {(['OR', 'AND'] as FinishCondition[]).map(opt => (
                  <button key={opt}
                    className={`condition-btn${leg.finishCondition === opt ? ' active' : ''}`}
                    onClick={() => onChange({ finishCondition: opt })}>
                    {opt}
                  </button>
                ))}
              </div>
            </div>
          )}

          {condCount === 0 && (
            <div className="field-hint" style={{ marginLeft: 0 }}>All off — leg drives but never advances</div>
          )}
        </section>

        <section>
          <div className="section-title">On Arrival</div>
          <label className="field-row">
            <span className="field-label">Intake</span>
            <select
              value={leg.intakeState}
              onChange={e => onChange({ intakeState: e.target.value as IntakeState })}
            >
              {INTAKE_STATES.map(s => <option key={s} value={s}>{s}</option>)}
            </select>
          </label>
          <label className="field-row">
            <span className="field-label">Flywheel</span>
            <select
              value={leg.flywheelState}
              onChange={e => onChange({ flywheelState: e.target.value as FlywheelState })}
            >
              {FLYWHEEL_STATES.map(s => <option key={s} value={s}>{s}</option>)}
            </select>
          </label>
        </section>
    </>
  );
}
