package conecalibrator

import (
	"context"
	"fmt"
	"math"

	"github.com/erh/vmodutils"
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
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

// planJoints plans all cone positions and returns the joint values (radians)
// for each stop. Does not move the arm.
func (s *coneCalibrator) planJoints(ctx context.Context) ([][]referenceframe.Input, error) {
	poses := computeConePositions(s.cfg.DistanceMM, s.cfg.RadiusMM, s.cfg.NumPositions)
	s.logger.Infof("computed %d sweep positions (including initial)", len(poses))

	fs, err := s.buildFrameSystem(ctx)
	if err != nil {
		return nil, err
	}

	startJoints, err := s.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("reading arm joint positions: %w", err)
	}

	s.logger.Info("planning all moves...")
	allJoints := [][]referenceframe.Input{startJoints}

	for i := 1; i < len(poses); i++ {
		goalPose := referenceframe.NewPoseInFrame(s.cfg.ArmName, poses[i])

		req := &armplanning.PlanRequest{
			FrameSystem: fs,
			Goals: []*armplanning.PlanState{
				armplanning.NewPlanState(
					referenceframe.FrameSystemPoses{s.cfg.ArmName: goalPose},
					nil,
				),
			},
			StartState: armplanning.NewPlanState(nil, referenceframe.FrameSystemInputs{
				s.cfg.ArmName: allJoints[len(allJoints)-1],
			}),
		}

		plan, _, err := armplanning.PlanMotion(ctx, s.logger, req)
		if err != nil {
			return nil, fmt.Errorf("motion plan failed for position %d/%d: %w", i, len(poses)-1, err)
		}

		traj := plan.Trajectory()
		if len(traj) < 2 {
			return nil, fmt.Errorf("unexpected trajectory length %d for position %d", len(traj), i)
		}
		goalJoints := traj[len(traj)-1][s.cfg.ArmName]
		allJoints = append(allJoints, goalJoints)

		s.logger.Infof("planned position %d/%d", i, len(poses)-1)
	}

	s.logger.Infof("all %d moves planned successfully", len(poses)-1)
	return allJoints, nil
}

// inputsToFloats converts [][]referenceframe.Input to [][]float64.
// In this RDK version, Input is a float64 alias, so this is a shallow copy.
func inputsToFloats(allJoints [][]referenceframe.Input) [][]float64 {
	result := make([][]float64, len(allJoints))
	for i, joints := range allJoints {
		vals := make([]float64, len(joints))
		copy(vals, joints)
		result[i] = vals
	}
	return result
}

// runSweep executes the full cone-scan calibration sweep.
func (s *coneCalibrator) runSweep(ctx context.Context) (map[string]interface{}, error) {
	allJoints, err := s.planJoints(ctx)
	if err != nil {
		return nil, err
	}

	// Execute moves and save calibration at each stop.
	s.logger.Info("saving calibration at initial position")
	if err := s.saveCalibrationPosition(ctx); err != nil {
		return nil, fmt.Errorf("saving calibration at initial position: %w", err)
	}
	saved := 1

	for i := 1; i < len(allJoints); i++ {
		s.logger.Infof("moving to position %d/%d", i, len(allJoints)-1)

		err := s.arm.MoveToJointPositions(ctx, allJoints[i], nil)
		if err != nil {
			return nil, fmt.Errorf("move to position %d failed: %w", i, err)
		}

		if err := s.saveCalibrationPosition(ctx); err != nil {
			return nil, fmt.Errorf("saving calibration at position %d: %w", i, err)
		}
		saved++
		s.logger.Infof("saved position %d/%d", i, len(allJoints)-1)
	}

	s.logger.Infof("sweep complete: %d positions saved", saved)
	return map[string]interface{}{
		"status":          "done",
		"positions_saved": saved,
	}, nil
}

func (s *coneCalibrator) handlePlanJoints(_ context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	if s.planning {
		s.mu.Unlock()
		return map[string]interface{}{"status": "already_planning"}, nil
	}
	s.planning = true
	s.planErr = nil
	s.mu.Unlock()

	go func() {
		err := s.planAndSave()

		s.mu.Lock()
		s.planning = false
		s.planErr = err
		s.mu.Unlock()

		if err != nil {
			s.logger.Errorf("plan_joints failed: %v", err)
		} else {
			s.logger.Info("plan_joints complete, config updated")
		}
	}()

	return map[string]interface{}{"status": "planning_started"}, nil
}

func (s *coneCalibrator) planAndSave() error {
	allJoints, err := s.planJoints(s.cancelCtx)
	if err != nil {
		return err
	}

	positions := inputsToFloats(allJoints)

	newAttr := utils.AttributeMap{
		"arm_name":            s.cfg.ArmName,
		"calibration_service": s.cfg.CalibrationService,
		"distance_mm":         s.cfg.DistanceMM,
		"radius_mm":           s.cfg.RadiusMM,
		"num_positions":       s.cfg.NumPositions,
		"planned_positions":   positions,
	}

	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(
		s.cancelCtx, s.name, newAttr, s.logger,
	)
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
