# CLAUDE.md — Cone Calibrator Module

## What This Is

Standalone Viam utility module that automates camera-arm calibration sweeps. Computes and executes a cone-scan pattern so the frame-calibration module gets diverse viewing angles of an apriltag sheet.

## Module Identity

- **Org namespace:** viamdemo
- **Module name:** cone-calibrator
- **Model triplet:** viamdemo:cone-calibrator:cone-calibrator
- **Model type:** Generic service

## Language

Go

## Project Layout

```
cone-calibrator/
  project_spec.md
  CLAUDE.md
  meta.json
  go.mod
  go.sum
  main.go
  cone_calibrator.go
```

## Key Conventions

- Go module, single binary output — no runtime dependencies on target
- Dependencies come from config attributes, resolved in `Reconfigure` via `Dependencies` map
- Use `resource.FromProvider`, never direct robot lookups
- Geometry math uses standard library `math`

## Development

```bash
go build -o cone-calibrator .
viam module reload-local --part-id <PART_ID> \
    --model-name viamdemo:cone-calibrator:cone-calibrator \
    --resource-name cone-calibrator
```

## Dependencies on Other Services

- **Arm:** To read current pose and as a motion target
- **Motion service (builtin):** For planning arm moves
- **Frame-calibration service:** To call `saveCalibrationPosition` and `checkTags` via `do_command`

## The Math

Compute N positions on a cone surface:
- Aim axis = current arm orientation (pointing at tag sheet)
- Circle of given radius at given distance along the aim axis
- At each position, arm orientation points back at sheet center
- Uses orthonormal basis perpendicular to aim axis
