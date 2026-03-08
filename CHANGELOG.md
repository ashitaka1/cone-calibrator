# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased]

### Added

- Cone-scan calibration sweep with configurable distance, radius, and number of positions
- DoCommand routing for `run_sweep` and `check_tags`
- Configurable attributes: `arm_name`, `calibration_service`, `distance_mm`, `radius_mm`, `num_positions`
- Two-phase sweep: plan all moves via `armplanning.PlanMotion`, then execute sequentially
- Calibration data saved at each sweep stop via frame-calibration service
- CLI entrypoint for standalone testing
