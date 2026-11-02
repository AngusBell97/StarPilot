package cereal

import (
	"bytes"
	"capnproto.org/go/capnp/v3"
	"encoding/base64"
	"encoding/json"
	"os"
	"pfeifer.dev/mapd/cereal/custom"
	"pfeifer.dev/mapd/cereal/log"
	"testing"
)

// This same actual Go wire fixture is decoded by the host Python schema tests.
func TestMapdV1HostWireFixture(t *testing.T) {
	msg, seg, err := capnp.NewMessage(capnp.SingleSegment(nil))
	if err != nil {
		t.Fatal(err)
	}
	event, err := log.NewRootEvent(seg)
	if err != nil {
		t.Fatal(err)
	}
	event.SetLogMonoTime(456000000000)
	event.SetValid(true)
	out, err := event.NewMapdOut()
	if err != nil {
		t.Fatal(err)
	}
	out.SetRoadName("Synthetic provider road")
	out.SetSpeedLimit(13.4112)
	out.SetTileLoaded(true)
	out.SetRoadContext(custom.RoadContext_city)
	out.SetWaySelectionType(custom.WaySelectionType_possible)
	out.SetHighwayClass(custom.HighwayClass_residential)
	out.SetWayId(1234567890123)
	out.SetConditionalSpeedLimit("20 @ (Mo-Fr 08:00-09:00)")
	out.SetSampleVersion(1)
	out.SetRoadStatus(custom.MapdOut_SampleStatus_matchedLimit)
	out.SetGpsSource(custom.MapdOut_GpsSource_external)
	out.SetGpsMonoTime(455950000000)
	out.SetComputedMonoTime(455990000000)
	out.SetSourceGeneration(3)
	out.SetProducerSession(0x1234abcd5011)
	raw, err := msg.Marshal()
	if err != nil {
		t.Fatal(err)
	}

	fixtureBytes, err := os.ReadFile("testdata/mapd_v1_wire.json")
	if err != nil {
		t.Fatal(err)
	}
	var fixture struct {
		EventBase64 string `json:"event_base64"`
	}
	if err := json.Unmarshal(fixtureBytes, &fixture); err != nil {
		t.Fatal(err)
	}
	expected, err := base64.StdEncoding.DecodeString(fixture.EventBase64)
	if err != nil {
		t.Fatal(err)
	}
    // Appended zero fields may enlarge allocation, but canonical old content
    // must remain byte-identical. Keep the original fixture as the old reader contract.
    oldMessage, err := capnp.Unmarshal(expected)
    if err != nil { t.Fatal(err) }
    oldEvent, err := log.ReadRootEvent(oldMessage)
    if err != nil { t.Fatal(err) }
    oldCanonical, err := capnp.Canonicalize(capnp.Struct(oldEvent))
    if err != nil { t.Fatal(err) }
    canonical, err := capnp.Canonicalize(capnp.Struct(event))
    if err != nil { t.Fatal(err) }
    if !bytes.Equal(canonical, oldCanonical) { t.Fatal("old schema field layout changed") }
    _ = raw
}

func TestTwoBoundaryHostWire(t *testing.T) {
 msg,seg,err:=capnp.NewMessage(capnp.SingleSegment(nil));if err!=nil{t.Fatal(err)}
 event,err:=log.NewRootEvent(seg);if err!=nil{t.Fatal(err)}
 event.SetValid(true);event.SetLogMonoTime(10_000_000_000)
 out,err:=event.NewMapdOut();if err!=nil{t.Fatal(err)}
 out.SetSampleVersion(2);out.SetRoadStatus(custom.MapdOut_SampleStatus_matchedLimit)
 out.SetGpsSource(custom.MapdOut_GpsSource_external);out.SetGpsMonoTime(9_900_000_000)
 out.SetComputedMonoTime(10_000_000_000);out.SetSourceGeneration(1);out.SetProducerSession(41)
 out.SetTileLoaded(true);out.SetWaySelectionType(custom.WaySelectionType_current);out.SetWayId(12)
 out.SetSpeedLimit(30);out.SetNextSpeedLimit(20);out.SetNextSpeedLimitDistance(100)
 out.SetNextSpeedLimitEndDistance(180);out.SetFollowingSpeedLimit(30)
 out.SetNextSpeedLimitWayId(22);out.SetFollowingSpeedLimitWayId(33)
 raw,err:=msg.Marshal();if err!=nil{t.Fatal(err)}
 if path:=os.Getenv("SP011_BOUNDARY_WIRE");path!="" {if err:=os.WriteFile(path,raw,0600);err!=nil{t.Fatal(err)}}
}
