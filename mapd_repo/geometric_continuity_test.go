package main

import (
	"pfeifer.dev/mapd/cereal"
	"testing"
)

func TestGeometricContinuityPreservesRealAmbiguity(t *testing.T) {
	current := syntheticWay{1, 13.4, 35.1, -97.9, 35.2, -97.9, false, 2}
	for _, tc := range []struct {
		name            string
		rival           syntheticWay
		speed, accuracy float32
		lat             float64
		matched         bool
	}{
		{"transverse away from crossing", syntheticWay{2, 20, 35.15008, -97.95, 35.15008, -97.85, false, 2}, 12, 3, 35.15, true},
		{"crossing at fix", syntheticWay{2, 20, 35.15, -97.95, 35.15, -97.85, false, 2}, 12, 3, 35.15, false},
		{"parallel road", syntheticWay{2, 20, 35.1, -97.8999, 35.2, -97.8999, false, 2}, 12, 3, 35.15, false},
		{"low speed heading uncertain", syntheticWay{2, 20, 35.15008, -97.95, 35.15008, -97.85, false, 2}, 2, 3, 35.15, false},
		{"large gps error", syntheticWay{2, 20, 35.15008, -97.95, 35.15008, -97.85, false, 2}, 12, 15, 35.15, false},
		{"connected past joint", syntheticWay{2, 20, 35.2, -97.9, 35.3, -97.9, false, 2}, 12, 3, 35.19993, true},
		{"joint within uncertainty", syntheticWay{2, 20, 35.2, -97.9, 35.3, -97.9, false, 2}, 12, 3, 35.19998, false},
	} {
		t.Run(tc.name, func(t *testing.T) {
			tile := packedRoadTile(t, []syntheticWay{current, tc.rival})
			sample := syntheticSample(t, cereal.GpsSourceExternal, 10000000000, 1, true, true)
			g := sample.Location
			g.SetLatitude(tc.lat)
			g.SetLongitude(-97.9)
			g.SetBearingDeg(0)
			g.SetSpeed(tc.speed)
			g.SetHorizontalAccuracy(tc.accuracy)
			if len(plausibleLoadedWays(&tile, g)) != 2 {
				t.Fatal("fixture does not reproduce broad ambiguity")
			}
			way, err := GetCurrentWay(CurrentWay{}, nil, &tile, g)
			if tc.matched {
				if err != nil || way.Way.Id() != 1 {
					t.Fatalf("lost supported interior road: %v %v", way.Way.Id(), err)
				}
			} else if err != ErrAmbiguousWay {
				t.Fatalf("resolved uncertain road: %v", err)
			}
		})
	}
}
