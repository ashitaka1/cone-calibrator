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
  Makefile
  go.mod
  go.sum
  module.go          # resource registration, Config, DoCommand, constructor
  cone.go            # geometry computation, sweep execution
  cone_test.go
  cmd/module/main.go # viam module entrypoint
  cmd/cli/main.go    # standalone CLI entrypoint
```

## Key Conventions

- Go module, single binary output — no runtime dependencies on target
- Dependencies come from config attributes, resolved in `Reconfigure` via `Dependencies` map
- Use typed `From` helpers (`arm.FromProvider`, `framesystem.FromDependencies`, `resource.FromDependencies[T]`)
- Geometry math uses `math` and `github.com/golang/geo/r3`; poses via `go.viam.com/rdk/spatialmath`
- Motion planning uses `armplanning.PlanMotion` (plan-then-execute pattern)
- `Config.Validate` returns `(deps []string, attrs []string, error)` — 3-return form

## Development

```bash
make                 # build to bin/cone-calibrator
make test            # go test ./...
make lint            # gofmt -s -w .
make module          # test + build tarball (module.tar.gz)
```

## Dependencies on Other Services

- **Arm:** To read current pose and as a motion target
- **Frame system service:** To build a FrameSystem for `armplanning.PlanMotion`
- **Frame-calibration service:** To call `saveCalibrationPosition` and `checkTags` via `do_command`

## The Math

Compute N poses in the arm's end-effector frame:
- Z axis = arm's aim direction (toward sheet), target = (0, 0, distance)
- Circle of given radius on the XY plane at Z=0
- At each position, orientation = normalized vector from position to target
- Two-phase execution: plan all moves via `armplanning.PlanMotion`, then execute sequentially
