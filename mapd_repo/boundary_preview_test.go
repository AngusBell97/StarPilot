package main

import (
	"pfeifer.dev/mapd/cereal"
	"pfeifer.dev/mapd/maps"
	m "pfeifer.dev/mapd/math"
	"testing"
	"time"
)

func TestShadowBoundaryPreviewProductionPipeline(t *testing.T) {
	for _, nextLimit := range []float64{8.9408, 26.8224} {
		for _, reverse := range []bool{false, true} {
			for _, fault := range []string{"", "junction", "unknown", "disconnected", "opposite_oneway"} {
				t.Run(fault+map[bool]string{true: "reverse", false: "forward"}[reverse]+time.Duration(nextLimit).String(), func(t *testing.T) {
					first := syntheticWay{1, 17.8816, 35.14, -97.9, 35.151, -97.9, false, 2}
					next := syntheticWay{2, nextLimit, 35.151, -97.9, 35.16, -97.9, false, 2}
					if reverse {
						first.lat1, first.lat2 = 35.149, 35.16
						next.lat1, next.lat2 = 35.14, 35.149
					}
					if fault == "unknown" {
						next.speed = 0
					}
					if fault == "disconnected" {
						next.lon1, next.lon2 = -97.89, -97.89
					}
					if fault == "opposite_oneway" {
						next.oneWay = true
						if !reverse {
							next.lat1, next.lat2 = next.lat2, next.lat1
						}
					}
					specs := []syntheticWay{first, next}
					if fault == "junction" {
						lat := 35.151
						if reverse {
							lat = 35.149
						}
						specs = append(specs, syntheticWay{3, 30, lat, -97.9, lat, -97.88, false, 2})
					}
					tile := packedRoadTile(t, specs)
					s := State{ShadowOnly: true}
					s.Init()
					boot := uint64(10_100_000_000)
					s.bootNow = func() uint64 { return boot }
					sample := syntheticSample(t, cereal.GpsSourceExternal, 10_000_000_000, 1, true, true)
					if reverse {
						sample.Location.SetBearingDeg(180)
					}
					load := func(m.Position) (maps.Offline, error) { return tile, nil }
					shadowOutput(t, &s, func() (cereal.GpsSample, bool) { return sample, true }, load, time.Unix(0, 0))
					sample.SourceChanged = false
					sample.FixMonoTime += 50_000_000
					boot += 50_000_000
					event, out := shadowOutput(t, &s, func() (cereal.GpsSample, bool) { return sample, true }, load, time.Unix(0, 0))
					if !event.Valid() {
						t.Fatal("current road unavailable", out.RoadStatus())
					}
					if fault == "" || fault == "junction" {
						if out.NextSpeedLimit() != float32(nextLimit) || out.NextSpeedLimitDistance() < 100 || out.NextSpeedLimitDistance() > 120 {
							t.Fatalf("missing/wrong boundary: %v %.2f", out.NextSpeedLimit(), out.NextSpeedLimitDistance())
						}
						if fault == "junction" && (out.NextSpeedLimitEndDistance() != 0 || out.FollowingSpeedLimit() != 0) {
							t.Fatal("predicted route must not qualify short-zone skipping")
						}
					} else if out.NextSpeedLimit() != 0 {
						t.Fatal("unqualified preview", out.NextSpeedLimit())
					}
					if out.SuggestedSpeed() != 0 || out.SpeedLimitSuggestedSpeed() != 0 || out.SpeedLimitAccepted() {
						t.Fatal("provider gained target authority")
					}
					boot = 10_600_000_000
					_, out = decodedOutput(t, &s)
					if out.NextSpeedLimit() != 0 || out.NextSpeedLimitDistance() != 0 {
						t.Fatal("stale preview survived")
					}
				})
			}
		}
	}
}

func TestTwoBoundaryPreviewStopsAtAmbiguity(t *testing.T) {
 for _, fault := range []string{"", "branch_after_first", "unknown_after_first"} {
  t.Run(fault, func(t *testing.T) {
   roads := []syntheticWay{
    {1, 17.8816, 35.14, -97.9, 35.151, -97.9, false, 2},
    {2, 8.9408, 35.151, -97.9, 35.1515, -97.9, false, 2},
    {3, 17.8816, 35.1515, -97.9, 35.16, -97.9, false, 2},
   }
   if fault == "branch_after_first" { roads=append(roads,syntheticWay{4, 20., 35.1515, -97.9, 35.1515, -97.88, false, 2}) }
   if fault == "unknown_after_first" { roads[2].speed=0 }
   tile:=packedRoadTile(t,roads);s:=State{ShadowOnly:true};s.Init()
   boot:=uint64(10_100_000_000);s.bootNow=func()uint64{return boot}
   sample:=syntheticSample(t,cereal.GpsSourceExternal,10_000_000_000,1,true,true)
   load:=func(m.Position)(maps.Offline,error){return tile,nil}
   shadowOutput(t,&s,func()(cereal.GpsSample,bool){return sample,true},load,time.Unix(0,0))
   sample.SourceChanged=false;sample.FixMonoTime+=50_000_000;boot+=50_000_000
   event,out:=shadowOutput(t,&s,func()(cereal.GpsSample,bool){return sample,true},load,time.Unix(0,0))
   if !event.Valid() || out.NextSpeedLimitWayId()!=2 || out.NextSpeedLimit()!=float32(8.9408) {t.Fatal("lost qualified first boundary")}
   if fault=="" {
    length:=out.NextSpeedLimitEndDistance()-out.NextSpeedLimitDistance()
    if length<54 || length>57 || out.FollowingSpeedLimitWayId()!=3 || out.FollowingSpeedLimit()!=float32(17.8816) {t.Fatal("wrong short-zone evidence",length,out.FollowingSpeedLimit())}
   } else if out.NextSpeedLimitEndDistance()!=0 || out.FollowingSpeedLimit()!=0 || out.FollowingSpeedLimitWayId()!=0 {t.Fatal("guessed following boundary")}
   boot=10_600_000_000;_,out=decodedOutput(t,&s)
   if out.NextSpeedLimitEndDistance()!=0 || out.FollowingSpeedLimit()!=0 || out.FollowingSpeedLimitWayId()!=0 || out.NextSpeedLimitWayId()!=0 {t.Fatal("stale second boundary survived")}
  })
 }
}
