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

// computeConePositions computes N poses arranged in a circle on a plane parallel to
// the calibration sheet, plus the initial dead-on pose. The arm's orientation at each
// position aims back at the sheet center.
//
// Geometry:
//   - P = current end-effector position
//   - A = aim axis (unit vector from the current orientation)
//   - Sheet center C = P + distance*A
//   - U, V = orthonormal basis perpendicular to A
//   - Position i: P + radius*(cos(angle)*U + sin(angle)*V)
//   - Orientation at each position: aim from that position toward C
func computeConePositions(currentPose spatialmath.Pose, distanceMM, radiusMM float64, n int) []spatialmath.Pose {
	p := currentPose.Point()
	ov := currentPose.Orientation().OrientationVectorRadians()
	aimAxis := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}.Normalize()

	sheetCenter := r3.Vector{
		X: p.X + distanceMM*aimAxis.X,
		Y: p.Y + distanceMM*aimAxis.Y,
		Z: p.Z + distanceMM*aimAxis.Z,
	}

	u, v := orthonormalBasis(aimAxis)

	poses := make([]spatialmath.Pose, 0, n+1)

	// Position 0: the initial dead-on pose (unchanged)
	poses = append(poses, currentPose)

	// Positions 1..N: circle around the aim axis
	for i := range n {
		angle := 2 * math.Pi * float64(i) / float64(n)
		cos, sin := math.Cos(angle), math.Sin(angle)

		pos := r3.Vector{
			X: p.X + radiusMM*(cos*u.X+sin*v.X),
			Y: p.Y + radiusMM*(cos*u.Y+sin*v.Y),
			Z: p.Z + radiusMM*(cos*u.Z+sin*v.Z),
		}

		dir := r3.Vector{
			X: sheetCenter.X - pos.X,
			Y: sheetCenter.Y - pos.Y,
			Z: sheetCenter.Z - pos.Z,
		}.Normalize()

		orientation := &spatialmath.OrientationVector{
			Theta: ov.Theta,
			OX:    dir.X,
			OY:    dir.Y,
			OZ:    dir.Z,
		}

		poses = append(poses, spatialmath.NewPose(pos, orientation))
	}

	return poses
}

// orthonormalBasis returns two unit vectors U, V perpendicular to the given axis,
// forming a right-handed coordinate system (U, V, axis).
func orthonormalBasis(axis r3.Vector) (r3.Vector, r3.Vector) {
	// Pick a vector not parallel to axis to seed the cross product
	seed := r3.Vector{X: 1, Y: 0, Z: 0}
	if math.Abs(axis.X) > 0.9 {
		seed = r3.Vector{X: 0, Y: 1, Z: 0}
	}

	u := axis.Cross(seed).Normalize()
	v := axis.Cross(u).Normalize()
	return u, v
}

// runSweep executes the full cone-scan calibration sweep.
// It computes all target positions, then moves through each sequentially,
// saving calibration data at every stop.
func (s *coneCalibrator) runSweep(ctx context.Context) (map[string]interface{}, error) {
	currentPose, err := s.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("reading arm position: %w", err)
	}

	poses := computeConePositions(currentPose, s.cfg.DistanceMM, s.cfg.RadiusMM, s.cfg.NumPositions)
	s.logger.Infof("computed %d sweep positions (including initial)", len(poses))

	saved := 0

	// Save calibration at the initial dead-on position (position 0)
	s.logger.Info("saving calibration at initial position")
	if err := s.saveCalibrationPosition(ctx); err != nil {
		return nil, fmt.Errorf("saving calibration at initial position: %w", err)
	}
	saved++

	// Move to each computed position and save calibration data
	for i := 1; i < len(poses); i++ {
		s.logger.Infof("moving to position %d/%d", i, len(poses)-1)

		dest := referenceframe.NewPoseInFrame("world", poses[i])
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
