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
2. The module reads the arm's current pose — this defines the aim axis.
3. Given the distance to the sheet, circle radius, and number of positions, it computes N poses on the surface of a cone:
   - The cone's apex is at the arm's current position
   - The cone's axis is the line from the arm to the center of the sheet
   - Each computed pose keeps the camera aimed at the center point
4. The module moves the arm to each pose using the motion service.
5. At each position, it calls `saveCalibrationPosition` on the frame-calibration service.

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

- Arm component (to read current pose, move via motion)
- Motion service (for planning moves)
- Frame-calibration service (to call `saveCalibrationPosition`)

## DoCommand Interface

### `run_sweep`

Runs the full cone-scan calibration sweep.

1. Reads current arm pose (the aim point)
2. Computes cone positions
3. Moves to each position, calling `saveCalibrationPosition` at each stop
4. Returns `{"status": "done", "positions_saved": N}`

### `check_tags`

Calls `checkTags` on the frame-calibration service at the current position. Returns the result. Useful for verifying the arm is aimed at the tags before starting.

## Tech Stack

- Go 1.21+
- `go.viam.com/rdk` — Viam Go SDK
- Standard library `math` package for cone geometry computation

## Math

The cone positions are computed as follows:
- Let **C** be the current end-effector position
- Let **A** be the aim axis (unit vector from C toward the sheet)
- The center of the circle on the sheet is **C + distance * A**
- Compute an orthonormal basis (U, V) perpendicular to A
- For each position i of N: angle = 2 * pi * i / N
  - Point on circle: **P_i = center + radius * (cos(angle) * U + sin(angle) * V)**
  - Arm pose: position orbits on a matching circle, orientation always points back at sheet center

The exact geometry needs careful implementation — the arm position moves on a circle at roughly constant distance from the sheet, and the orientation vector at each position always points to the sheet center.

## Build & Deploy

- `go build` produces a single static binary — no runtime dependencies on target
- `meta.json` entrypoint points to the binary
- Deploy with `viam module reload-local`

## Viam SDK Patterns (Go)

- Dependencies come from config, resolved in `Reconfigure` via the `Dependencies` map
- Use `resource.FromProvider` to look up typed resources
- `do_command` on the frame-calibration service is how you call `saveCalibrationPosition` and `checkTags`
