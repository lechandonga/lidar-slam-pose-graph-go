package slam

import (
	"bytes"
	"encoding/gob"
)

// keyframeWire is the stable on-disk representation of a Keyframe. It is a
// superset of the historical v1 "keyframeGob" layout: gob matches fields by
// name, so a v1 file (which lacks Session) decodes into this type with
// Session left at zero, where migrateV1ToV2 assigns it to session 0.
//
// Keyframe historically carried a custom GobEncode/GobDecode that wrapped
// this struct in a nested gob stream. Keeping the custom codec here means
// genuinely old files produced by the previous program still decode, while
// new files are written in the same self-describing wire format.
type keyframeWire struct {
	ID                          int
	FrameID                     int64
	Time                        float64
	Pose, PoseOdom, SurfaceBase Pose
	Session                     int
	Scan, Surface               []Point
	UpdatedAt                   int64
}

// GobEncode implements a stable wire format for Keyframe.
func (k *Keyframe) GobEncode() ([]byte, error) {
	var buf bytes.Buffer
	if err := gob.NewEncoder(&buf).Encode(keyframeWire{
		ID: k.ID, FrameID: k.FrameID, Time: k.Time,
		Pose: k.Pose, PoseOdom: k.PoseOdom, SurfaceBase: k.SurfaceBase,
		Session: k.Session, Scan: k.Scan, Surface: k.Surface,
		UpdatedAt: k.UpdatedAt,
	}); err != nil {
		return nil, err
	}
	return buf.Bytes(), nil
}

// GobDecode accepts both the current keyframeWire stream and the historical
// v1 keyframeGob stream (field-name matched; Session defaults to zero).
func (k *Keyframe) GobDecode(data []byte) error {
	var w keyframeWire
	if err := gob.NewDecoder(bytes.NewReader(data)).Decode(&w); err != nil {
		return err
	}
	*k = Keyframe{
		ID: w.ID, FrameID: w.FrameID, Time: w.Time,
		Pose: w.Pose, PoseOdom: w.PoseOdom, SurfaceBase: w.SurfaceBase,
		Session: w.Session, Scan: w.Scan, Surface: w.Surface,
		UpdatedAt: w.UpdatedAt,
	}
	return nil
}
