package conecalibrator

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
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
	ArmName            string  `json:"arm_name"`
	CalibrationService string  `json:"calibration_service"`
	DistanceMM         float64 `json:"distance_mm"`
	RadiusMM           float64 `json:"radius_mm"`
	NumPositions       int     `json:"num_positions"`
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

	arm       arm.Arm
	motion    motion.Service
	calSvc    resource.Resource
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

	mot, err := motion.FromProvider(deps, "builtin")
	if err != nil {
		return nil, fmt.Errorf("motion service: %w", err)
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
		motion:    mot,
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
	return nil, fmt.Errorf("unknown command, expected run_sweep or check_tags")
}

func (s *coneCalibrator) Close(context.Context) error {
	s.cancelFn()
	return nil
}
