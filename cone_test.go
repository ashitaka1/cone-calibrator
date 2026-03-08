package conecalibrator

import (
	"math"
	"testing"

	"go.viam.com/rdk/spatialmath"
)

func TestComputeConePositions(t *testing.T) {
	distanceMM := 500.0
	radiusMM := 100.0
	numPositions := 4

	poses := computeConePositions(distanceMM, radiusMM, numPositions)

	if len(poses) != numPositions+1 {
		t.Fatalf("expected %d poses, got %d", numPositions+1, len(poses))
	}

	// Position 0 should be the zero pose (identity)
	p0 := poses[0].Point()
	if p0.X != 0 || p0.Y != 0 || p0.Z != 0 {
		t.Errorf("position 0 should be origin, got %v", p0)
	}

	for i := 1; i < len(poses); i++ {
		pt := poses[i].Point()

		// All orbit positions should be at Z=0 (XY plane)
		if math.Abs(pt.Z) > 1e-9 {
			t.Errorf("position %d: expected Z=0, got Z=%f", i, pt.Z)
		}

		// All orbit positions should be at the configured radius
		dist := math.Sqrt(pt.X*pt.X + pt.Y*pt.Y)
		if math.Abs(dist-radiusMM) > 1e-9 {
			t.Errorf("position %d: expected radius %f, got %f", i, radiusMM, dist)
		}

		// Orientation should aim at the target (0, 0, distance)
		ov := poses[i].Orientation().OrientationVectorRadians()
		dir := spatialmath.OrientationVector{OX: ov.OX, OY: ov.OY, OZ: ov.OZ}
		expectedX := -pt.X
		expectedY := -pt.Y
		expectedZ := distanceMM
		norm := math.Sqrt(expectedX*expectedX + expectedY*expectedY + expectedZ*expectedZ)

		if math.Abs(dir.OX-expectedX/norm) > 1e-6 ||
			math.Abs(dir.OY-expectedY/norm) > 1e-6 ||
			math.Abs(dir.OZ-expectedZ/norm) > 1e-6 {
			t.Errorf("position %d: orientation (%f,%f,%f) doesn't aim at target, expected (%f,%f,%f)",
				i, dir.OX, dir.OY, dir.OZ, expectedX/norm, expectedY/norm, expectedZ/norm)
		}
	}
}

func TestComputeConePositionsSpacing(t *testing.T) {
	poses := computeConePositions(500, 100, 8)

	// Check angular spacing between consecutive orbit positions
	for i := 1; i < len(poses)-1; i++ {
		p1 := poses[i].Point()
		p2 := poses[i+1].Point()
		a1 := math.Atan2(p1.Y, p1.X)
		a2 := math.Atan2(p2.Y, p2.X)
		diff := a2 - a1
		if diff < 0 {
			diff += 2 * math.Pi
		}
		expected := 2 * math.Pi / 8.0
		if math.Abs(diff-expected) > 1e-9 {
			t.Errorf("angular gap between positions %d and %d: expected %f, got %f", i, i+1, expected, diff)
		}
	}
}
