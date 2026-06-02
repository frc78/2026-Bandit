# Auto Visualizer

A local web tool for visually creating and editing FRC autonomous routines, outputting Kotlin code compatible with the robot codebase.

## Prerequisites

- **Node.js 18 or newer** — download from https://nodejs.org (LTS version recommended)
  - Verify: `node --version` and `npm --version`

That's it. No other global installs needed.

## Setup

From the repo root:

```bash
cd auto-tool
npm install
```

## Running

```bash
npm run dev
```

Then open **http://localhost:5173** in a browser.

The dev server must stay running while you use the tool — it handles all file reads and writes. `Ctrl+C` stops it.

---

## Creating an auto

### 1. Name your auto

Set the name in the top bar. This becomes the Kotlin object name and the filename, so use **PascalCase** (e.g. `FourPieceCenterLine`).

### 2. Set the start pose

With no leg selected, the right panel shows the **Start Pose** editor. Set the robot's starting X/Y position (in meters, WPILib blue alliance coordinates — origin at the blue driver station corner) and rotation (degrees, CCW from the +x axis / pointing toward the red alliance).

You can also set the **initial subsystem states** here: intake and flywheel state at the very beginning of the auto.

The green **S** marker on the field represents the start pose. Drag it to reposition, or drag the green circle handle to rotate.

### 3. Add legs

Click **+** in the leg list (left panel) to add a waypoint. Each leg represents a pose the robot drives to. They execute in order.

**On the field:**
- **Drag** a robot marker to reposition the waypoint
- **Drag the circle handle** above the marker to set rotation (hold Shift to snap to 15°)
- **Click** a marker to select it for editing

**In the leg list:**
- **Drag the ⠿ handle** to reorder legs
- **⧉** duplicates a leg (inserted immediately after, with `Copy` appended to the name)
- **×** deletes a leg

### 4. Edit leg parameters

Select a leg to open its editor in the right panel.

#### Identity
- **Name** — PascalCase; used as the Kotlin `Step` enum entry and variable name. Must be unique.

#### Target Pose
- **X / Y** — destination in meters
- **Rotation** — heading at the destination in degrees (CCW from +x)
- **Intake First** — when checked, the robot faces its intake toward the target while driving (rotation is auto-computed and locked). Useful for intaking notes on the way to a waypoint.

#### Drive Parameters
- **Max Speed** — cap in m/s. Check "Robot max" to use the robot's configured top speed.
- **Always Max Speed** — skip the distance-based slowdown near the target; useful when a timer is the finish condition instead of a threshold.
- **Min Speed** — floor in m/s, so the robot never crawls too slowly near the target.
- **Max Rotation** — cap in °/s. Check "Robot max" to use the robot's configured top rotation rate.

#### Finish Parameters

The leg advances to the next step when its finish condition(s) are met. At least one must be enabled, or the leg drives forever.

- **Threshold** — advance when the robot is within this distance (meters) of the target
- **Leg timer** — advance after this many seconds since the current leg started
- **Auto timer** — advance after this many seconds since the auto started (useful for hard cutoffs)

When two or more are enabled, use **OR** (whichever fires first) or **AND** (all must be true simultaneously).

#### On Arrival
- **Intake / Flywheel** — set subsystem state when the finish condition is met. `Unchanged` leaves the current state alone.

### 5. Save

Click **Save** in the top bar. Two files are written:

```
auto-tool/autos/<AutoName>.json                          ← GUI source of truth
src/main/java/frc/robot/auto/gui_tests/<AutoName>.kt     ← generated Kotlin
```

The JSON is what the GUI re-loads. The Kotlin goes into `gui_tests/` for review — move it to the main `auto/` package when it's ready.

### 6. Load an existing auto

If any `.json` files are present in `auto-tool/autos/`, a dropdown and **Load** button appear in the top bar. Select an auto and click Load. You'll be prompted if there are unsaved changes.

---

## Simulator

The simulator lets you preview an auto's motion and subsystem state changes without recompiling the robot code. It's useful for catching mistakes in finish parameters, speed settings, and timing before a full sim run.

### Controls

| Control | Description |
|---|---|
| ▶ / ⏸ | Play / pause |
| ↺ | Reset to start pose |
| 0.5× 1× 2× 5× | Playback speed multiplier |
| `0.0s` | Auto elapsed time |

### What you see

- **Orange robot** — the simulated robot position, drawn on top of the design waypoints
- **Gold ring** around a waypoint — the leg currently being driven to
- **Gold row** in the leg list — same active leg highlighted in the list
- **State badges** in the top bar — current effective intake and flywheel state

### Motion model

The sim uses a simplified linear model — no PID, no acceleration ramp. Translation speed is clamped between `minSpeed` and `maxSpeed`, scaling down within ~1m of the target (unless `alwaysMaxSpeed` is on). Rotation lerps at up to 360°/s. This means elapsed time will read slightly lower than real auto time for fast or long legs; the sim is calibrated to catch **logic** mistakes (wrong threshold, timer fires too early, AND vs OR, intakeFirst pointing wrong way) rather than provide exact timing.

Editing the auto while the sim is running resets it automatically.

---

## Coordinate system

All coordinates use **WPILib blue alliance field coordinates**:
- Origin at the blue driver station corner
- +X toward the red alliance, +Y toward the left wall (from blue driver perspective)
- Rotation in degrees, CCW from +X

The field image is calibrated to match. When loading a pre-existing auto, make sure the start pose was recorded in blue alliance coordinates.
