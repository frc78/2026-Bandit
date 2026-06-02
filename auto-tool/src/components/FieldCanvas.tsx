import { useEffect, useRef, useState } from 'react';
import { Stage, Layer, Image as KonvaImage, Group, Rect, Arrow, Text, Circle } from 'react-konva';
import type { AutoConfig, IntakeState } from '../types';
import { ROBOT_LENGTH_M, ROBOT_WIDTH_M, INTAKE_EXT_M } from '../constants';

// Image dimensions (landscape, blue on left)
const IMG_W = 1057;
const IMG_H = 779;
const IMG_ASPECT = IMG_W / IMG_H; // 1.357

// Calibration — two known field→pixel pairs:
//   Field (0, 0)            → image pixel (30, 731)
//   Field (8.2705, 8.069)   → image pixel (732, 46)
const CAL_ORIGIN = { x: 30, y: 731 }; // image pixel of WPILib (0, 0)
const CAL_SCALE  =
  ((732 - 30) / 8.2705 + (731 - 46) / 8.069) / 2; // avg ≈ 84.89 px/m

interface SimOverlay {
  pose: { x: number; y: number; rotationDeg: number };
  legIndex: number;
}

interface Props {
  auto: AutoConfig;
  selectedId: string | null;
  simOverlay: SimOverlay | null;
  onSelect: (id: string | null) => void;
  onLegMove: (id: string, x: number, y: number) => void;
  onLegRotate: (id: string, deg: number) => void;
  onStartMove: (x: number, y: number) => void;
  onStartRotate: (deg: number) => void;
}

function useFieldImage(src: string): HTMLImageElement | null {
  const [img, setImg] = useState<HTMLImageElement | null>(null);
  useEffect(() => {
    const el = new Image();
    el.src = src;
    el.onload = () => setImg(el);
  }, [src]);
  return img;
}

export default function FieldCanvas({
  auto,
  selectedId,
  simOverlay,
  onSelect,
  onLegMove,
  onLegRotate,
  onStartMove,
  onStartRotate,
}: Props) {
  const containerRef = useRef<HTMLDivElement>(null);
  const stageRef = useRef<any>(null);
  const [size, setSize] = useState({ w: 400, h: 543 });
  const fieldImage = useFieldImage('/auto_field.png');

  // Rotation tracking — no Konva drag, just mousedown on handle + mousemove on stage
  const rotating = useRef<{ id: string | 'start'; cx: number; cy: number } | null>(null);

  // Release rotation if mouse is released anywhere (including outside the stage)
  useEffect(() => {
    const onUp = () => { rotating.current = null; };
    window.addEventListener('mouseup', onUp);
    return () => window.removeEventListener('mouseup', onUp);
  }, []);

  // Fit canvas within container while preserving image aspect ratio
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const ro = new ResizeObserver(([entry]) => {
      const maxW = entry.contentRect.width;
      const maxH = entry.contentRect.height;
      let w = maxW;
      let h = w / IMG_ASPECT;
      if (h > maxH) {
        h = maxH;
        w = h * IMG_ASPECT;
      }
      setSize({ w: Math.floor(w), h: Math.floor(h) });
    });
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  const { w, h } = size;

  // Canvas px/m = image px/m scaled to canvas size
  const mToP = CAL_SCALE * (w / IMG_W);

  /**
   * WPILib field coords (meters) → Konva canvas pixels.
   * Uses calibrated origin + uniform scale; y-axis is inverted (field up = screen up).
   */
  function f2c(xM: number, yM: number) {
    return {
      x: (CAL_ORIGIN.x + CAL_SCALE * xM) * (w / IMG_W),
      y: (CAL_ORIGIN.y - CAL_SCALE * yM) * (h / IMG_H),
    };
  }

  /** Konva canvas pixels → WPILib field coords */
  function c2f(cx: number, cy: number) {
    const imgX = cx * (IMG_W / w);
    const imgY = cy * (IMG_H / h);
    return {
      x: (imgX - CAL_ORIGIN.x) / CAL_SCALE,
      y: (CAL_ORIGIN.y - imgY) / CAL_SCALE,
    };
  }

  /**
   * WPILib rotation (degrees, CCW from +x) → Konva rotation (degrees, CW from screen-up).
   * Blue on left, y-axis inverted: konva = 90 - wpiLibDeg
   */
  function toKonvaRot(deg: number): number {
    return 90 - deg;
  }

  function round3(v: number) { return Math.round(v * 1000) / 1000; }
  function round1(v: number) { return Math.round(v * 10) / 10; }

  /** Compute WPILib degrees from robot center to a screen point, with optional 15° snap. */
  function angleFromCenter(cx: number, cy: number, px: number, py: number, snap: boolean) {
    const screenAngle = Math.atan2(px - cx, -(py - cy)) * (180 / Math.PI);
    const wpiDeg = 90 - screenAngle;
    return snap ? Math.round(wpiDeg / 15) * 15 : round1(wpiDeg);
  }

  function handleStageMouseMove(e: any) {
    if (!rotating.current) return;
    const pos = stageRef.current?.getPointerPosition();
    if (!pos) return;
    const { id, cx, cy } = rotating.current;
    const deg = angleFromCenter(cx, cy, pos.x, pos.y, e.evt.shiftKey);
    if (id === 'start') onStartRotate(deg);
    else onLegRotate(id, deg);
  }

  const robotW = ROBOT_WIDTH_M * mToP;
  const robotH = ROBOT_LENGTH_M * mToP;
  const intakeH = INTAKE_EXT_M * mToP;

  const allWaypoints = [
    { x: auto.startPose.x, y: auto.startPose.y },
    ...auto.legs.map(l => ({ x: l.x, y: l.y })),
  ];

  return (
    <div ref={containerRef} className="field-container">
      <Stage ref={stageRef} width={w} height={h} onClick={() => onSelect(null)} onMouseMove={handleStageMouseMove}>
        <Layer>
          {/* Field background */}
          {fieldImage && (
            <KonvaImage image={fieldImage} x={0} y={0} width={w} height={h} />
          )}

          {/* Path arrows */}
          {allWaypoints.slice(0, -1).map((from, i) => {
            const to = allWaypoints[i + 1];
            const a = f2c(from.x, from.y);
            const b = f2c(to.x, to.y);
            return (
              <Arrow
                key={i}
                points={[a.x, a.y, b.x, b.y]}
                stroke="rgba(255,255,255,0.55)"
                fill="rgba(255,255,255,0.55)"
                strokeWidth={2}
                pointerLength={10}
                pointerWidth={8}
                dash={[10, 5]}
                listening={false}
              />
            );
          })}

          {/* Threshold circles */}
          {auto.legs.filter(l => l.finishThreshold && l.thresholdM > 0).map(leg => {
            const pos = f2c(leg.x, leg.y);
            return (
              <Circle
                key={`thresh-${leg.id}`}
                x={pos.x} y={pos.y}
                radius={leg.thresholdM * mToP}
                stroke="rgba(255,255,0,0.85)"
                strokeWidth={2}
                dash={[4, 4]}
                listening={false}
              />
            );
          })}

          {/* Start pose */}
          {(() => {
            const pos = f2c(auto.startPose.x, auto.startPose.y);
            return (
              <Group
                x={pos.x} y={pos.y}
                rotation={toKonvaRot(auto.startPose.rotationDeg)}
                draggable={!simOverlay}
                onDragStart={e => { e.cancelBubble = true; }}
                onDragEnd={e => {
                  const p = c2f(e.target.x(), e.target.y());
                  onStartMove(round3(p.x), round3(p.y));
                }}
                onClick={e => { e.cancelBubble = true; onSelect(null); }}
              >
                <Rect x={-robotW / 2} y={-robotH / 2}
                  width={robotW} height={robotH}
                  fill="rgba(80,200,80,0.35)" stroke="#4c4" strokeWidth={2} cornerRadius={3} />
                <Arrow points={[0, -robotH / 2 + 4, 0, -robotH / 2 - intakeH / 2]}
                  stroke="#4c4" fill="#4c4" strokeWidth={2} pointerLength={7} pointerWidth={7} />
                <Text text="S" x={0} y={0}
                  width={16} align="center"
                  offsetX={8} offsetY={6.5}
                  rotation={-(toKonvaRot(auto.startPose.rotationDeg))}
                  fill="#4c4" fontSize={13} fontStyle="bold" />
                {!simOverlay && (
                  <Circle
                    x={0} y={-(robotH / 2 + intakeH + 14)}
                    radius={7}
                    fill="#4c4" stroke="white" strokeWidth={1.5}
                    onMouseDown={e => {
                      e.cancelBubble = true;
                      const gp = e.target.parent!.getAbsolutePosition();
                      rotating.current = { id: 'start', cx: gp.x, cy: gp.y };
                    }}
                  />
                )}
              </Group>
            );
          })()}

          {/* Leg waypoints */}
          {(() => {
            // Compute effective intake state for each leg by propagating forward
            const effectiveIntake: IntakeState[] = [];
            let lastIntake: IntakeState = auto.startPose.intakeState;
            for (const leg of auto.legs) {
              if (leg.intakeState !== 'Unchanged') lastIntake = leg.intakeState;
              effectiveIntake.push(lastIntake);
            }
            return auto.legs.map((leg, i) => {
            const pos = f2c(leg.x, leg.y);
            const isSelected = leg.id === selectedId;
            const isActiveLeg = simOverlay !== null && simOverlay.legIndex === i;
            const eff = effectiveIntake[i];
            const showIntake = eff === 'Intaking' || eff === 'Deployed';
            return (
              <Group
                key={leg.id}
                x={pos.x} y={pos.y}
                rotation={toKonvaRot(leg.rotationDeg)}
                draggable={!simOverlay}
                onDragStart={e => { e.cancelBubble = true; onSelect(leg.id); }}
                onDragEnd={e => {
                  const p = c2f(e.target.x(), e.target.y());
                  onLegMove(leg.id, round3(p.x), round3(p.y));
                }}
                onClick={e => { e.cancelBubble = true; onSelect(leg.id); }}
              >
                {showIntake && (
                  <Rect x={-robotW / 2} y={-robotH / 2 - intakeH}
                    width={robotW} height={intakeH}
                    fill="rgba(255,140,0,0.5)" stroke="rgba(255,140,0,1.0)"
                    strokeWidth={2} dash={[3, 3]} />
                )}
                {isActiveLeg && (
                  <Circle x={0} y={0}
                    radius={Math.max(robotW, robotH) / 2 + 8}
                    stroke="#ffd700" strokeWidth={3}
                    fill="transparent" listening={false} />
                )}
                <Rect x={-robotW / 2} y={-robotH / 2}
                  width={robotW} height={robotH}
                  fill={isSelected ? 'rgba(80,170,255,0.75)' : 'rgba(80,140,220,0.5)'}
                  stroke={isSelected ? '#6df' : '#6af'}
                  strokeWidth={isSelected ? 3 : 2}
                  cornerRadius={3} />
                <Arrow points={[0, -robotH / 2 + 4, 0, -robotH / 2 - intakeH / 2]}
                  stroke={isSelected ? '#fff' : '#adf'}
                  fill={isSelected ? '#fff' : '#adf'}
                  strokeWidth={2} pointerLength={7} pointerWidth={7} />
                <Text text={String(i + 1)} x={0} y={0}
                  width={16} align="center"
                  offsetX={8} offsetY={6.5}
                  rotation={-(toKonvaRot(leg.rotationDeg))}
                  fill="white" fontSize={13} fontStyle="bold" />
                <Circle
                  x={0} y={-(robotH / 2 + intakeH + 14)}
                  radius={7}
                  fill={isSelected ? '#6df' : 'rgba(100,200,255,0.5)'}
                  stroke="white" strokeWidth={1.5}
                  onMouseDown={e => {
                    e.cancelBubble = true;
                    onSelect(leg.id);
                    const gp = e.target.parent!.getAbsolutePosition();
                    rotating.current = { id: leg.id, cx: gp.x, cy: gp.y };
                  }}
                  onClick={e => { e.cancelBubble = true; }}
                />
              </Group>
            );
          });
          })()}


          {/* Sim robot overlay */}
          {simOverlay && (() => {
            const pos = f2c(simOverlay.pose.x, simOverlay.pose.y);
            return (
              <Group
                x={pos.x} y={pos.y}
                rotation={toKonvaRot(simOverlay.pose.rotationDeg)}
                listening={false}
              >
                <Rect x={-robotW / 2} y={-robotH / 2}
                  width={robotW} height={robotH}
                  fill="rgba(255,140,0,0.75)" stroke="#ff8c00"
                  strokeWidth={2} cornerRadius={3} />
                <Arrow points={[0, -robotH / 2 + 4, 0, -robotH / 2 - intakeH / 2]}
                  stroke="#fff" fill="#fff"
                  strokeWidth={2} pointerLength={7} pointerWidth={7} />
              </Group>
            );
          })()}
        </Layer>
      </Stage>
    </div>
  );
}
