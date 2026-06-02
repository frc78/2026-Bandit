import { useState, useCallback, useEffect, useRef } from 'react';
import { v4 as uuidv4 } from 'uuid';
import type { AutoConfig, Leg, IntakeState, FlywheelState } from './types';
import { MAX_SPEED_MS } from './constants';

// ── Simulator ────────────────────────────────────────────────────────────────

interface SimState {
  running: boolean;
  pose: { x: number; y: number; rotationDeg: number };
  legIndex: number;    // 0..n-1 = active leg, n = finished
  autoElapsed: number; // seconds since sim started
  legElapsed: number;  // seconds since current leg started
  speed: number;       // playback multiplier: 1 | 2 | 5
}

function lerpAngle(from: number, to: number, maxDelta: number): number {
  let diff = (to - from) % 360;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  if (Math.abs(diff) <= maxDelta) return to;
  return from + Math.sign(diff) * maxDelta;
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

function initSimState(startPose: { x: number; y: number; rotationDeg: number }, speed = 1): SimState {
  return { running: false, pose: { ...startPose }, legIndex: 0, autoElapsed: 0, legElapsed: 0, speed };
}

function tickSim(prev: SimState, auto: AutoConfig): SimState {
  if (!prev.running) return prev;
  const { legs } = auto;
  if (prev.legIndex >= legs.length) return { ...prev, running: false };

  const leg = legs[prev.legIndex];
  const dt = 0.02 * prev.speed;
  let { x, y, rotationDeg } = prev.pose;

  const diffX = leg.x - x;
  const diffY = leg.y - y;
  const distance = Math.hypot(diffX, diffY);

  if (distance > 0.001) {
    const maxSpd = leg.maxSpeed ?? MAX_SPEED_MS;
    const spd = leg.alwaysMaxSpeed ? maxSpd : clamp(maxSpd * (distance / 1.0), leg.minSpeed, maxSpd);
    const move = Math.min(spd * dt, distance);
    x += (diffX / distance) * move;
    y += (diffY / distance) * move;
  }

  const targetRot = leg.intakeFirst
    ? Math.atan2(diffY, diffX) * (180 / Math.PI)
    : leg.rotationDeg;
  rotationDeg = lerpAngle(rotationDeg, targetRot, 360 * dt);

  const newAutoElapsed = prev.autoElapsed + dt;
  const newLegElapsed = prev.legElapsed + dt;

  const conds: boolean[] = [];
  if (leg.finishThreshold) conds.push(Math.hypot(leg.x - x, leg.y - y) <= leg.thresholdM);
  if (leg.finishLegTimer !== null) conds.push(newLegElapsed >= leg.finishLegTimer);
  if (leg.finishAutoTimer !== null) conds.push(newAutoElapsed >= leg.finishAutoTimer);

  const advance = conds.length > 0 && (
    leg.finishCondition === 'AND' ? conds.every(Boolean) : conds.some(Boolean)
  );

  if (advance) {
    const nextIndex = prev.legIndex + 1;
    return {
      ...prev,
      pose: { x: leg.x, y: leg.y, rotationDeg: leg.rotationDeg },
      legIndex: nextIndex,
      autoElapsed: newAutoElapsed,
      legElapsed: 0,
      running: nextIndex < legs.length,
    };
  }

  return { ...prev, pose: { x, y, rotationDeg }, autoElapsed: newAutoElapsed, legElapsed: newLegElapsed };
}
import FieldCanvas from './components/FieldCanvas';
import LegList from './components/LegList';
import LegEditor from './components/LegEditor';
import { generateKotlin } from './codeGen';
import './App.css';

function makeLeg(name: string, x = 5.0, y = 4.0): Leg {
  return {
    id: uuidv4(),
    name,
    x,
    y,
    rotationDeg: 0,
    thresholdM: 0.3,
    maxSpeed: null,
    maxRotation: null,
    alwaysMaxSpeed: false,
    minSpeed: 0,
    intakeFirst: false,
    finishThreshold: true,
    finishLegTimer: null,
    finishAutoTimer: null,
    finishCondition: 'OR',
    intakeState: 'Unchanged',
    flywheelState: 'Unchanged',
  };
}

const DEFAULT_AUTO: AutoConfig = {
  name: 'NewAuto',
  startPose: {
    x: 3.36,
    y: 5.737,
    rotationDeg: 0,
    intakeState: 'Retracted',
    flywheelState: 'Unchanged',
  },
  legs: [],
};

export default function App() {
  const [auto, setAuto] = useState<AutoConfig>(DEFAULT_AUTO);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [saveMsg, setSaveMsg] = useState('');
  const [isDirty, setIsDirty] = useState(false);
  const [autoList, setAutoList] = useState<string[]>([]);
  const [selectedLoad, setSelectedLoad] = useState('');
  const [simState, setSimState] = useState<SimState>(() => initSimState(DEFAULT_AUTO.startPose));
  const autoRef = useRef(auto);

  // Keep autoRef in sync so the interval can always see the latest auto
  useEffect(() => { autoRef.current = auto; }, [auto]);

  // Reset sim when auto changes while running
  useEffect(() => {
    setSimState(prev => {
      if (!prev.running) return prev;
      return { ...initSimState(auto.startPose), speed: prev.speed };
    });
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [auto]);

  // Tick interval — only active when running
  useEffect(() => {
    if (!simState.running) return;
    const id = setInterval(() => {
      setSimState(prev => tickSim(prev, autoRef.current));
    }, 20);
    return () => clearInterval(id);
  }, [simState.running]);

  const handleSimPlay = () => {
    setSimState(prev => ({ ...prev, running: !prev.running }));
  };

  const handleSimReset = () => {
    setSimState(prev => ({ ...initSimState(auto.startPose), speed: prev.speed }));
  };

  const handleSimSpeed = (s: number) => {
    setSimState(prev => ({ ...prev, speed: s }));
  };

  const simActive = simState.running || simState.autoElapsed > 0;
  const simIntakeState: IntakeState = (() => {
    let state = auto.startPose.intakeState;
    for (let i = 0; i < simState.legIndex && i < auto.legs.length; i++) {
      if (auto.legs[i].intakeState !== 'Unchanged') state = auto.legs[i].intakeState;
    }
    return state;
  })();
  const simFlywheelState: FlywheelState = (() => {
    let state = auto.startPose.flywheelState;
    for (let i = 0; i < simState.legIndex && i < auto.legs.length; i++) {
      if (auto.legs[i].flywheelState !== 'Unchanged') state = auto.legs[i].flywheelState;
    }
    return state;
  })();

  useEffect(() => {
    fetch('/api/list-autos')
      .then(r => r.json())
      .then((names: string[]) => { setAutoList(names); if (names.length) setSelectedLoad(names[0]); })
      .catch(() => {});
  }, []);

  const selectedLeg = auto.legs.find(l => l.id === selectedId) ?? null;
  const selectedIndex = auto.legs.findIndex(l => l.id === selectedId);

  const updateLeg = useCallback((id: string, updates: Partial<Leg>) => {
    setAuto(prev => ({
      ...prev,
      legs: prev.legs.map(l => (l.id === id ? { ...l, ...updates } : l)),
    }));
    setIsDirty(true);
  }, []);

  const addLeg = useCallback(() => {
    const leg = makeLeg(`Step${auto.legs.length + 1}`);
    setAuto(prev => ({ ...prev, legs: [...prev.legs, leg] }));
    setSelectedId(leg.id);
    setIsDirty(true);
  }, [auto.legs.length]);

  const deleteLeg = useCallback((id: string) => {
    setAuto(prev => ({ ...prev, legs: prev.legs.filter(l => l.id !== id) }));
    if (selectedId === id) setSelectedId(null);
    setIsDirty(true);
  }, [selectedId]);

  const duplicateLeg = useCallback((id: string) => {
    setAuto(prev => {
      const idx = prev.legs.findIndex(l => l.id === id);
      if (idx === -1) return prev;
      const src = prev.legs[idx];
      const copy = { ...src, id: uuidv4(), name: `${src.name}Copy` };
      const next = [...prev.legs];
      next.splice(idx + 1, 0, copy);
      return { ...prev, legs: next };
    });
    setIsDirty(true);
  }, []);

  const reorderLeg = useCallback((fromIndex: number, toIndex: number) => {
    setAuto(prev => {
      const next = [...prev.legs];
      const [item] = next.splice(fromIndex, 1);
      next.splice(toIndex, 0, item);
      return { ...prev, legs: next };
    });
    setIsDirty(true);
  }, []);

  const handleSave = async () => {
    let code: string;
    try {
      code = generateKotlin(auto);
    } catch (e) {
      setSaveMsg(e instanceof Error ? e.message : 'Validation error');
      return;
    }
    try {
      const res = await fetch('/api/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name: auto.name, code, autoJson: JSON.stringify(auto, null, 2) }),
      });
      if (res.ok) {
        setIsDirty(false);
        setSaveMsg('Saved!');
        setTimeout(() => setSaveMsg(''), 3000);
        fetch('/api/list-autos')
          .then(r => r.json())
          .then((names: string[]) => { setAutoList(names); setSelectedLoad(auto.name); })
          .catch(() => {});
      } else {
        setSaveMsg('Error saving');
      }
    } catch {
      setSaveMsg('Server error');
    }
  };

  const handleLoad = async () => {
    if (!selectedLoad) return;
    if (isDirty && !window.confirm('You have unsaved changes. Load anyway?')) return;
    try {
      const res = await fetch(`/api/load-auto/${encodeURIComponent(selectedLoad)}`);
      if (!res.ok) { setSaveMsg('Load failed'); return; }
      const loaded: AutoConfig = await res.json();
      setAuto(loaded);
      setSelectedId(null);
      setIsDirty(false);
      setSaveMsg('');
      const names: string[] = await fetch('/api/list-autos').then(r => r.json());
      setAutoList(names);
      setSelectedLoad(loaded.name);
    } catch {
      setSaveMsg('Load error');
    }
  };

  const updateStart = useCallback(
    (field: string, value: number | IntakeState | FlywheelState) => {
      setAuto(prev => ({
        ...prev,
        startPose: { ...prev.startPose, [field]: value },
      }));
      setIsDirty(true);
    },
    []
  );

  return (
    <div className="app">
      <header className="top-bar">
        <span className="app-title">Auto Visualizer</span>
        <label className="name-field">
          Auto name:
          <input
            value={auto.name}
            onChange={e => { setAuto(prev => ({ ...prev, name: e.target.value })); setIsDirty(true); }}
            placeholder="MyAuto"
          />
        </label>
        {autoList.length > 0 && (
          <>
            <select
              className="auto-select"
              value={selectedLoad}
              onChange={e => setSelectedLoad(e.target.value)}
            >
              {autoList.map(n => <option key={n} value={n}>{n}</option>)}
            </select>
            <button className="btn-load" onClick={handleLoad}>Load</button>
          </>
        )}
        <div className="sim-controls">
          <button
            className={`btn-sim-play${simState.running ? ' active' : ''}`}
            onClick={handleSimPlay}
            title={simState.running ? 'Pause' : 'Play'}
            disabled={auto.legs.length === 0}
          >
            {simState.running ? '⏸' : '▶'}
          </button>
          <button className="btn-sim-reset" onClick={handleSimReset} title="Reset">↺</button>
          <select
            className="sim-speed-select"
            value={simState.speed}
            onChange={e => handleSimSpeed(Number(e.target.value))}
          >
            {[0.5, 1, 2, 5].map(s => <option key={s} value={s}>{s}×</option>)}
          </select>
          <span className="sim-elapsed">{simState.autoElapsed.toFixed(1)}s</span>
          {simActive && (
            <span className="sim-states">
              <span className={`sim-state-badge intake-${simIntakeState.toLowerCase()}`}>
                {simIntakeState}
              </span>
              <span className={`sim-state-badge flywheel-${simFlywheelState.toLowerCase()}`}>
                {simFlywheelState}
              </span>
            </span>
          )}
        </div>
        <button className="btn-save" onClick={handleSave}>
          Save
        </button>
        {saveMsg && <span className={`save-msg ${saveMsg !== 'Saved!' ? 'err' : ''}`}>{saveMsg}</span>}
      </header>

      <div className="main-area">
        <LegList
          legs={auto.legs}
          selectedId={selectedId}
          activeLegIndex={simActive && simState.legIndex < auto.legs.length ? simState.legIndex : null}
          onSelect={setSelectedId}
          onAdd={addLeg}
          onDelete={deleteLeg}
          onDuplicate={duplicateLeg}
          onReorder={reorderLeg}
        />

        <FieldCanvas
          auto={auto}
          selectedId={selectedId}
          simOverlay={simActive ? { pose: simState.pose, legIndex: simState.legIndex } : null}
          onSelect={setSelectedId}
          onLegMove={(id, x, y) => {
            setAuto(prev => {
              const idx = prev.legs.findIndex(l => l.id === id);
              const leg = prev.legs[idx];
              if (!leg) return prev;
              const updates: Partial<Leg> = { x, y };
              if (leg.intakeFirst) {
                const pp = idx === 0 ? prev.startPose : prev.legs[idx - 1];
                updates.rotationDeg = Math.atan2(y - pp.y, x - pp.x) * (180 / Math.PI);
              }
              return { ...prev, legs: prev.legs.map(l => l.id === id ? { ...l, ...updates } : l) };
            });
            setIsDirty(true);
          }}
          onLegRotate={(id, deg) => updateLeg(id, { rotationDeg: deg })}
          onStartMove={(x, y) => setAuto(prev => ({
            ...prev,
            startPose: { ...prev.startPose, x, y },
          }))}
          onStartRotate={deg => setAuto(prev => ({
            ...prev,
            startPose: { ...prev.startPose, rotationDeg: deg },
          }))}
        />

        <div className="panel leg-editor">
          <div className="panel-header">
            {selectedLeg ? `Leg ${selectedIndex + 1}: ${selectedLeg.name}` : 'Start Pose'}
          </div>
          <div className="editor-body">
            {selectedLeg ? (
              <LegEditor
                leg={selectedLeg}
                index={selectedIndex}
                prevPose={selectedIndex === 0 ? auto.startPose : auto.legs[selectedIndex - 1]}
                onChange={updates => updateLeg(selectedLeg.id, updates)}
              />
            ) : (
              <StartPoseEditor auto={auto} onChange={updateStart} />
            )}
          </div>
        </div>
      </div>
    </div>
  );
}

function StartPoseEditor({
  auto,
  onChange,
}: {
  auto: AutoConfig;
  onChange: (field: string, value: number | IntakeState | FlywheelState) => void;
}) {
  const { startPose: sp } = auto;
  return (
    <>
      <section>
        <div className="section-title">Start Pose</div>
        <label className="field-row">
          <span className="field-label">X</span>
          <div className="field-input-wrap">
            <input type="number" step="0.01" value={sp.x}
              onChange={e => onChange('x', parseFloat(e.target.value))} />
            <span className="field-unit">m</span>
          </div>
        </label>
        <label className="field-row">
          <span className="field-label">Y</span>
          <div className="field-input-wrap">
            <input type="number" step="0.01" value={sp.y}
              onChange={e => onChange('y', parseFloat(e.target.value))} />
            <span className="field-unit">m</span>
          </div>
        </label>
        <label className="field-row">
          <span className="field-label">Rotation</span>
          <div className="field-input-wrap">
            <input type="number" step="1" value={sp.rotationDeg}
              onChange={e => onChange('rotationDeg', parseFloat(e.target.value))} />
            <span className="field-unit">°</span>
          </div>
        </label>
      </section>
      <section>
        <div className="section-title">Initial State</div>
        <label className="field-row">
          <span className="field-label">Intake</span>
          <select value={sp.intakeState}
            onChange={e => onChange('intakeState', e.target.value as IntakeState)}>
            {(['Unchanged', 'Retracted', 'Intaking', 'Deployed', 'Jostle'] as IntakeState[]).map(s =>
              <option key={s} value={s}>{s}</option>)}
          </select>
        </label>
        <label className="field-row">
          <span className="field-label">Flywheel</span>
          <select value={sp.flywheelState}
            onChange={e => onChange('flywheelState', e.target.value as FlywheelState)}>
            {(['Unchanged', 'Still', 'Firing'] as FlywheelState[]).map(s =>
              <option key={s} value={s}>{s}</option>)}
          </select>
        </label>
      </section>
      <div className="field-hint start-hint">
        Select a leg in the list or click a waypoint on the field to edit it. Drag waypoints to reposition them.
      </div>
    </>
  );
}
