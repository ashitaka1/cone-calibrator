package conecalibrator

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// computeConePositions computes N poses in the arm's end-effector frame arranged
// in a circle on the XY plane, plus a zero pose for the initial position.
//
// In the arm's frame:
//   - Z axis = the arm's aim direction (toward the sheet)
//   - Target point = (0, 0, distance_mm) — the sheet center
//   - Circle positions = (radius*cos, radius*sin, 0) on the XY plane
//   - Orientation at each position = normalized vector from position to target
func computeConePositions(distanceMM, radiusMM float64, n int) []spatialmath.Pose {
	poses := make([]spatialmath.Pose, 0, n+1)

	// Position 0: identity pose (the arm stays where it is)
	poses = append(poses, spatialmath.NewZeroPose())

	target := r3.Vector{X: 0, Y: 0, Z: distanceMM}

	for i := range n {
		angle := 2 * math.Pi * float64(i) / float64(n)
		x := radiusMM * math.Cos(angle)
		y := radiusMM * math.Sin(angle)

		pos := r3.Vector{X: x, Y: y, Z: 0}

		// Aim from this position toward the target point
		dir := target.Sub(pos).Normalize()

		orientation := &spatialmath.OrientationVector{
			OX: dir.X,
			OY: dir.Y,
			OZ: dir.Z,
		}

		poses = append(poses, spatialmath.NewPose(pos, orientation))
	}

	return poses
}

// runSweep executes the full cone-scan calibration sweep.
// It computes target positions in the arm's frame, then moves through each
// sequentially via the motion service, saving calibration data at every stop.
func (s *coneCalibrator) runSweep(ctx context.Context) (map[string]interface{}, error) {
	poses := computeConePositions(s.cfg.DistanceMM, s.cfg.RadiusMM, s.cfg.NumPositions)
	s.logger.Infof("computed %d sweep positions (including initial)", len(poses))

	saved := 0

	// Save calibration at the initial dead-on position
	s.logger.Info("saving calibration at initial position")
	if err := s.saveCalibrationPosition(ctx); err != nil {
		return nil, fmt.Errorf("saving calibration at initial position: %w", err)
	}
	saved++

	// Move to each computed position and save calibration data.
	// Poses are in the arm's frame — the motion service handles the transform.
	for i := 1; i < len(poses); i++ {
		s.logger.Infof("moving to position %d/%d", i, len(poses)-1)

		dest := referenceframe.NewPoseInFrame(s.cfg.ArmName, poses[i])
		_, err := s.motion.Move(ctx, motion.MoveReq{
			ComponentName: s.cfg.ArmName,
			Destination:   dest,
		})
		if err != nil {
			return nil, fmt.Errorf("move to position %d failed (saved %d so far): %w", i, saved, err)
		}

		if err := s.saveCalibrationPosition(ctx); err != nil {
			return nil, fmt.Errorf("saving calibration at position %d: %w", i, err)
		}
		saved++
		s.logger.Infof("saved position %d/%d", i, len(poses)-1)
	}

	s.logger.Infof("sweep complete: %d positions saved", saved)
	return map[string]interface{}{
		"status":          "done",
		"positions_saved": saved,
	}, nil
}

func (s *coneCalibrator) saveCalibrationPosition(ctx context.Context) error {
	_, err := s.calSvc.DoCommand(ctx, map[string]interface{}{
		"save_calibration_position": true,
	})
	return err
}

func (s *coneCalibrator) checkTags(ctx context.Context) (map[string]interface{}, error) {
	return s.calSvc.DoCommand(ctx, map[string]interface{}{
		"check_tags": true,
	})
}
