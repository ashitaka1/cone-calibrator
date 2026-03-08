package conecalibrator

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/spatialmath"
)

func TestComputeConePositions(t *testing.T) {
	// Arm at origin, pointing along +Z
	startPose := spatialmath.NewPose(
		r3.Vector{X: 0, Y: 0, Z: 0},
		&spatialmath.OrientationVector{Theta: 0, OX: 0, OY: 0, OZ: 1},
	)

	distanceMM := 500.0
	radiusMM := 100.0
	numPositions := 4

	poses := computeConePositions(startPose, distanceMM, radiusMM, numPositions)

	// Should have N+1 positions (initial + N orbit positions)
	if len(poses) != numPositions+1 {
		t.Fatalf("expected %d poses, got %d", numPositions+1, len(poses))
	}

	// Position 0 should be the original pose
	p0 := poses[0].Point()
	if p0.X != 0 || p0.Y != 0 || p0.Z != 0 {
		t.Errorf("position 0 should be origin, got %v", p0)
	}

	sheetCenter := r3.Vector{X: 0, Y: 0, Z: distanceMM}

	for i := 1; i < len(poses); i++ {
		pt := poses[i].Point()

		// All orbit positions should be at Z=0 (same depth as start)
		if math.Abs(pt.Z) > 1e-9 {
			t.Errorf("position %d: expected Z=0, got Z=%f", i, pt.Z)
		}

		// All orbit positions should be at radius distance from origin in XY plane
		dist := math.Sqrt(pt.X*pt.X + pt.Y*pt.Y)
		if math.Abs(dist-radiusMM) > 1e-9 {
			t.Errorf("position %d: expected radius %f, got %f", i, radiusMM, dist)
		}

		// Orientation should aim at the sheet center
		ov := poses[i].Orientation().OrientationVectorRadians()
		dir := r3.Vector{X: ov.OX, Y: ov.OY, Z: ov.OZ}.Normalize()
		expected := r3.Vector{
			X: sheetCenter.X - pt.X,
			Y: sheetCenter.Y - pt.Y,
			Z: sheetCenter.Z - pt.Z,
		}.Normalize()

		dot := dir.X*expected.X + dir.Y*expected.Y + dir.Z*expected.Z
		if math.Abs(dot-1.0) > 1e-6 {
			t.Errorf("position %d: orientation not aimed at sheet center (dot=%f)", i, dot)
		}
	}
}

func TestComputeConePositionsSpacing(t *testing.T) {
	// Verify positions are evenly spaced around the circle
	startPose := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.OrientationVector{Theta: 0, OX: 0, OY: 0, OZ: 1},
	)

	poses := computeConePositions(startPose, 500, 100, 8)
	origin := startPose.Point()

	// Check angular spacing between consecutive orbit positions
	for i := 1; i < len(poses)-1; i++ {
		p1 := poses[i].Point()
		p2 := poses[i+1].Point()
		a1 := math.Atan2(p1.Y-origin.Y, p1.X-origin.X)
		a2 := math.Atan2(p2.Y-origin.Y, p2.X-origin.X)
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

func TestOrthonormalBasis(t *testing.T) {
	axes := []r3.Vector{
		{X: 0, Y: 0, Z: 1},
		{X: 1, Y: 0, Z: 0},
		{X: 0, Y: 1, Z: 0},
		{X: 1, Y: 1, Z: 1},
	}

	for _, axis := range axes {
		axis = axis.Normalize()
		u, v := orthonormalBasis(axis)

		// U and V should be unit vectors
		if math.Abs(u.Norm()-1) > 1e-9 {
			t.Errorf("axis %v: U not unit length: %f", axis, u.Norm())
		}
		if math.Abs(v.Norm()-1) > 1e-9 {
			t.Errorf("axis %v: V not unit length: %f", axis, v.Norm())
		}

		// All three should be mutually orthogonal
		if math.Abs(u.Dot(v)) > 1e-9 {
			t.Errorf("axis %v: U·V = %f (should be 0)", axis, u.Dot(v))
		}
		if math.Abs(u.Dot(axis)) > 1e-9 {
			t.Errorf("axis %v: U·axis = %f (should be 0)", axis, u.Dot(axis))
		}
		if math.Abs(v.Dot(axis)) > 1e-9 {
			t.Errorf("axis %v: V·axis = %f (should be 0)", axis, v.Dot(axis))
		}
	}
}
