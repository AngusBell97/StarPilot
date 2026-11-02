package main

import (
	"encoding/json"
	"os"
	"pfeifer.dev/mapd/cereal"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
	ms "pfeifer.dev/mapd/settings"
	"testing"
	"time"
)

func TestRecordedContinuityReplay(t *testing.T) {
	old := ms.Settings
	defer func() { ms.Settings = old }()
	if err := initShadowDefaults(); err != nil {
		t.Fatal(err)
	}
	input, root, output := os.Getenv("SP011_GPS"), os.Getenv("SP011_MAPS"), os.Getenv("SP011_OUTPUT")
	if input == "" {
		t.Skip("private local GPS/map replay is explicit")
	}
	raw, err := os.ReadFile(input)
	if err != nil {
		t.Fatal(err)
	}
	var rows []struct {
		T                                             uint64
		Valid                                         bool
		Latitude, Longitude, Bearing, Speed, Accuracy float64
		HasFix                                        bool
	}
	if err = json.Unmarshal(raw, &rows); err != nil {
		t.Fatal(err)
	}
	s := State{ShadowOnly: true}
	s.Init()
	var boot uint64
	s.bootNow = func() uint64 { return boot }
	load := func(p m.Position) (maps.Offline, error) { return maps.FindWaysAroundPositionIn(p, root) }
	var results []map[string]any
	previews := 0
	for i, row := range rows {
		sample := syntheticSample(t, cereal.GpsSourceExternal, row.T, 1, i == 0, true)
		sample.Location.SetLatitude(row.Latitude)
		sample.Location.SetLongitude(row.Longitude)
		sample.Location.SetBearingDeg(float32(row.Bearing))
		sample.Location.SetHorizontalAccuracy(float32(row.Accuracy))
		sample.Location.SetHasFix(row.HasFix)
		sample.Location.SetSpeed(float32(row.Speed))
		boot = row.T + 10_000_000
		event, out := shadowOutput(t, &s, func() (cereal.GpsSample, bool) { return sample, row.Valid && row.HasFix }, load, time.Unix(0, int64(row.T)))
		if out.NextSpeedLimit() > 0 {
			previews++
		}
		candidates := []map[string]any{}
		for _, w := range plausibleLoadedWays(&s.Data, sample.Location) {
			n, _ := w.OnWay(sample.Location, w.DistanceMultiplier())
			a, _ := w.BearingAlignment(sample.Location)
			candidates = append(candidates, map[string]any{"id": w.Id(), "speed": w.MaxSpeed(), "narrow": n.OnWay, "distance": n.Distance.Distance, "projection": n.Distance.LinePosition.T, "alignment": a})
		}
		results = append(results, map[string]any{"candidates": candidates, "way": out.WayId(), "t": float64(row.T) / 1e9, "valid": event.Valid(), "status": out.RoadStatus().String(), "selection": out.WaySelectionType().String(), "current": out.SpeedLimit(), "next": out.NextSpeedLimit(), "distance": out.NextSpeedLimitDistance()})
	}
	data, _ := json.MarshalIndent(results, "", "  ")
	if err = os.WriteFile(output, data, 0600); err != nil {
		t.Fatal(err)
	}
	t.Logf("%d fixes, %d preview samples", len(rows), previews)
	if len(results) == 0 {
		t.Fatal("no upcoming limits recovered")
	}
}
