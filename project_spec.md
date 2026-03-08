# Project Specification — Cone Calibrator Module

## Purpose

A standalone Viam module that automates camera-arm calibration by sweeping the arm through computed positions on a cone pattern. The arm maintains aim at a fixed point (the center of an apriltag calibration sheet) while orbiting around it, collecting calibration data at each position.

## User Profile

Anyone calibrating a wrist-mounted camera on a robotic arm. Run once per hardware setup to determine the camera-to-arm transform.

## Goals

**Goal:** Given an arm positioned facing a calibration sheet, automatically compute and execute a cone-scan pattern that provides the frame-calibration module with diverse viewing angles.

**Goal:** Expose a simple `do_command` interface — one command to run the sweep.

**Non-Goal:** Running the frame-calibration solve itself. This module moves the arm and calls `saveCalibrationPosition`; the user runs `runCalibration` separately on the frame-calibration service.

## How It Works

1. The user manually positions the arm so it faces the apriltag sheet dead-on.
2. The module reads the arm's current joint positions — this defines the starting state.
3. Given the distance to the sheet, circle radius, and number of positions, it computes N poses in the arm's end-effector frame:
   - Z axis = arm's aim direction (toward the sheet)
   - Target point = (0, 0, distance_mm) — the sheet center
   - Circle positions = (radius*cos, radius*sin, 0) on the XY plane
   - Orientation at each position = normalized vector from position to target
4. **Phase 1 (Plan):** Plans all moves sequentially via `armplanning.PlanMotion`, feeding each plan's end joints as the next plan's start. This validates feasibility before any motion occurs.
5. **Phase 2 (Execute):** Moves to each planned joint position via `arm.MoveToJointPositions`, calling `saveCalibrationPosition` at each stop.

## Module Identity

- **Org namespace:** viamdemo
- **Module name:** cone-calibrator
- **Model triplet:** viamdemo:cone-calibrator:cone-calibrator
- **Model type:** Generic service

## Configuration Attributes

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| `arm_name` | string | yes | Name of the arm component |
| `calibration_service` | string | yes | Name of the frame-calibration service |
| `distance_mm` | float | yes | Distance from arm to calibration sheet |
| `radius_mm` | float | yes | Radius of the circle at the sheet |
| `num_positions` | int | yes | Number of positions around the circle |

## Dependencies

- Arm component (to read joint positions, execute planned moves)
- Frame system service (to build a FrameSystem for `armplanning.PlanMotion`)
- Frame-calibration service (to call `saveCalibrationPosition` via DoCommand)

## DoCommand Interface

### `run_sweep`

Runs the full cone-scan calibration sweep.

1. Computes cone positions in the arm's end-effector frame
2. Plans all moves via `armplanning.PlanMotion` (validates feasibility)
3. Executes each move via `arm.MoveToJointPositions`
4. Calls `saveCalibrationPosition` at each stop
5. Returns `{"status": "done", "positions_saved": N}`

### `check_tags`

Calls `checkTags` on the frame-calibration service at the current position. Returns the result. Useful for verifying the arm is aimed at the tags before starting.

## Tech Stack

- Go 1.25+
- `go.viam.com/rdk` v0.115.0 — Viam Go SDK
- `github.com/golang/geo/r3` — 3D vector math
- `go.viam.com/rdk/motionplan/armplanning` — motion planning
- `go.viam.com/rdk/spatialmath` — pose and orientation types

## Project Layout

```
cone-calibrator/
  module.go          # resource registration, Config, DoCommand, constructor
  cone.go            # geometry computation, sweep execution
  cone_test.go       # unit tests
  cmd/module/main.go # viam module entrypoint
  cmd/cli/main.go    # standalone CLI entrypoint
  Makefile
  meta.json
```

## Technical Architecture

### Two-Phase Sweep (Plan-Then-Execute)

The sweep separates planning from execution for safety:
1. All N moves are planned sequentially, each starting from the previous plan's end joints
2. If any plan fails, no motion has occurred — the arm stays in place
3. Only after all plans succeed does execution begin

### Frame System Construction

`buildFrameSystem` fetches the full frame system config from the framesystem service, filters to only the arm's parts, and constructs a minimal FrameSystem. This is passed to `armplanning.PlanMotion`.

### Dependency Resolution

Uses typed `From` helpers in the constructor:
- `arm.FromProvider(deps, name)` for the arm
- `framesystem.FromDependencies(deps)` for the frame system service
- `resource.FromDependencies[resource.Resource](deps, name)` for the calibration service (generic)

## Math

Poses are computed in the arm's end-effector frame:
- Z axis = arm's aim direction (toward sheet)
- Target = (0, 0, distance_mm) — the sheet center
- For each position i of N: angle = 2*pi*i/N
  - Position = (radius*cos(angle), radius*sin(angle), 0)
  - Orientation = normalized vector from position to target
- Position 0 is the identity pose (arm stays where it is)

## Build & Deploy

- `make` builds to `bin/cone-calibrator`
- `make module.tar.gz` builds the deployable tarball
- Target architectures: linux/amd64, linux/arm64, darwin/arm64, windows/amd64
- `meta.json` entrypoint: `bin/cone-calibrator`

## Viam SDK Patterns (Go)

- Dependencies come from config, resolved in `Reconfigure` via the `Dependencies` map
- Use typed `From` helpers (`arm.FromProvider`, `framesystem.FromDependencies`, `resource.FromDependencies[T]`)
- `do_command` on the frame-calibration service is how you call `saveCalibrationPosition` and `checkTags`
- `Config.Validate` returns `(deps []string, attrs []string, error)` — 3-return form

## Milestones

- [x] Module scaffold
- [x] Cone geometry computation
- [x] Two-phase sweep (plan-then-execute)
- [ ] Hardware validation
