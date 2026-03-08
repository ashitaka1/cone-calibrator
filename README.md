# Cone Calibrator Module

Viam module that automates camera-arm calibration sweeps. Computes and executes a cone-scan pattern so a frame-calibration service gets diverse viewing angles of an apriltag sheet.

**Model:** `viamdemo:cone-calibrator:cone-calibrator` (Generic service)

## When to Use

You have a wrist-mounted camera on a robotic arm and need to calibrate the camera-to-arm transform. Position the arm facing an apriltag calibration sheet, and this module sweeps through computed positions while collecting calibration data.

## Prerequisites

- An arm component configured in Viam
- A frame-calibration service (exposes `saveCalibrationPosition` and `checkTags` via DoCommand)
- An apriltag calibration sheet visible to the camera

## Configuration

```json
{
  "arm_name": "my-arm",
  "calibration_service": "frame-calibration",
  "distance_mm": 500,
  "radius_mm": 100,
  "num_positions": 8
}
```

| Attribute | Type | Required | Description |
|-----------|------|----------|-------------|
| `arm_name` | string | yes | Name of the arm component |
| `calibration_service` | string | yes | Name of the frame-calibration service |
| `distance_mm` | float | yes | Distance from arm to calibration sheet (mm) |
| `radius_mm` | float | yes | Radius of the orbit circle (mm) |
| `num_positions` | int | yes | Number of positions around the circle |

## Usage

1. **Position the arm** so it faces the apriltag sheet dead-on.

2. **Verify tags are visible** (optional):
   ```json
   {"check_tags": true}
   ```

3. **Run the sweep**:
   ```json
   {"run_sweep": true}
   ```
   Returns: `{"status": "done", "positions_saved": N}`

4. **Run calibration** separately on the frame-calibration service.

## Building

```bash
make                  # build to bin/cone-calibrator
make test             # run tests
make module.tar.gz    # build deployable tarball
```

For local development:
```bash
viam module reload-local --part-id <PART_ID> \
    --model-name viamdemo:cone-calibrator:cone-calibrator \
    --resource-name cone-calibrator
```
