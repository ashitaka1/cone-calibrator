package conecalibrator

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/generic"
)

var ConeCalibrator = resource.NewModel("viamdemo", "cone-calibrator", "cone-calibrator")

func init() {
	resource.RegisterService(generic.API, ConeCalibrator,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newConeCalibratorConeCalibrator,
		},
	)
}

type Config struct {
	ArmName            string      `json:"arm_name"`
	CalibrationService string      `json:"calibration_service"`
	DistanceMM         float64     `json:"distance_mm"`
	RadiusMM           float64     `json:"radius_mm"`
	NumPositions       int         `json:"num_positions"`
	PlannedPositions   [][]float64 `json:"planned_positions,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.ArmName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm_name")
	}
	if cfg.CalibrationService == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "calibration_service")
	}
	if cfg.DistanceMM <= 0 {
		return nil, nil, fmt.Errorf("%s: distance_mm must be positive", path)
	}
	if cfg.RadiusMM <= 0 {
		return nil, nil, fmt.Errorf("%s: radius_mm must be positive", path)
	}
	if cfg.NumPositions <= 0 {
		return nil, nil, fmt.Errorf("%s: num_positions must be positive", path)
	}

	deps := []string{
		cfg.ArmName,
		cfg.CalibrationService,
	}
	return deps, nil, nil
}

type coneCalibrator struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	arm    arm.Arm
	rfs    framesystem.Service
	calSvc resource.Resource

	mu       sync.Mutex
	planning bool
	planErr  error

	cancelCtx context.Context
	cancelFn  func()
}

func newConeCalibratorConeCalibrator(
	ctx context.Context,
	deps resource.Dependencies,
	rawConf resource.Config,
	logger logging.Logger,
) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewConeCalibrator(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewConeCalibrator(
	ctx context.Context,
	deps resource.Dependencies,
	name resource.Name,
	conf *Config,
	logger logging.Logger,
) (resource.Resource, error) {
	a, err := arm.FromProvider(deps, conf.ArmName)
	if err != nil {
		return nil, fmt.Errorf("arm %q: %w", conf.ArmName, err)
	}

	rfs, err := framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("frame system: %w", err)
	}

	calSvc, err := resource.FromDependencies[resource.Resource](deps, generic.Named(conf.CalibrationService))
	if err != nil {
		return nil, fmt.Errorf("calibration service %q: %w", conf.CalibrationService, err)
	}

	cancelCtx, cancelFn := context.WithCancel(context.Background())

	return &coneCalibrator{
		name:      name,
		logger:    logger,
		cfg:       conf,
		arm:       a,
		rfs:       rfs,
		calSvc:    calSvc,
		cancelCtx: cancelCtx,
		cancelFn:  cancelFn,
	}, nil
}

func (s *coneCalibrator) Name() resource.Name {
	return s.name
}

func (s *coneCalibrator) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["run_sweep"]; ok {
		return s.runSweep(ctx)
	}
	if _, ok := cmd["check_tags"]; ok {
		return s.checkTags(ctx)
	}
	if _, ok := cmd["plan_joints"]; ok {
		return s.handlePlanJoints(ctx)
	}
	if _, ok := cmd["planning_status"]; ok {
		return s.planningStatus()
	}
	return nil, fmt.Errorf("unknown command, expected run_sweep, check_tags, plan_joints, or planning_status")
}

func (s *coneCalibrator) planningStatus() (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	if s.planning {
		return map[string]interface{}{"status": "planning"}, nil
	}
	if s.planErr != nil {
		return map[string]interface{}{"status": "error", "error": s.planErr.Error()}, nil
	}
	if len(s.cfg.PlannedPositions) > 0 {
		return map[string]interface{}{
			"status":           "done",
			"joint_positions":  s.cfg.PlannedPositions,
			"num_positions":    len(s.cfg.PlannedPositions),
		}, nil
	}
	return map[string]interface{}{"status": "idle"}, nil
}

func (s *coneCalibrator) Close(context.Context) error {
	s.cancelFn()
	return nil
}

// buildFrameSystem constructs a FrameSystem from the robot's full frame config.
// The arm's frame config may reference a static offset frame (its parent),
// so we include all parts to ensure the tree resolves correctly.
func (s *coneCalibrator) buildFrameSystem(ctx context.Context) (*referenceframe.FrameSystem, error) {
	fsCfg, err := s.rfs.FrameSystemConfig(ctx)
	if err != nil {
		return nil, fmt.Errorf("getting frame system config: %w", err)
	}

	fs, err := referenceframe.NewFrameSystem("cone-calibrator", fsCfg.Parts, nil)
	if err != nil {
		return nil, fmt.Errorf("building frame system: %w", err)
	}
	return fs, nil
}
