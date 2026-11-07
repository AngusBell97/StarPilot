"""Saved climb options: old documents read as Off, Off is not written, On round-trips and reaches the policy."""

from dataclasses import replace
import json
import unittest

from openpilot.starpilot.conditional_mode.preferences import (
  CCMOptions, PreferenceError, SavedPreferences, decode_preferences, encode_preferences,
)
from openpilot.starpilot.conditional_mode.policy import ModeChoice

OLD = (b'{"ccm":{"launch_assist":false,"lead":true,"persist_manual":false,"set_speed_margin_mps":0.44704,"speed_mps":0.0,'
       b'"speed_with_lead_mps":0.0},"cem":{"curves":false,"curves_with_lead":false,"lead":true,"model_stop_s":7.7,'
       b'"open_road":true,"persist_manual":false,"signal_lane_detection":true,"signal_lane_width_m":1.524,'
       b'"signal_speed_mps":44.25696,"slower_lead":true,"speed_mps":0.0,"speed_with_lead_mps":0.0,"stop_lights":true,'
       b'"stopped_lead":true},"mode":"stock","version":1}')


class ClimbSettingsTests(unittest.TestCase):
  def test_the_saved_comma_document_reads_with_climb_off(self):
    prefs = decode_preferences(OLD)
    self.assertEqual((prefs.ccm.climb, prefs.ccm.climb_on_bend_exit, prefs.ccm.climb_lead), (False, False, False))
    self.assertEqual(encode_preferences(prefs), OLD)  # nothing new is written while every option is Off

  def test_on_round_trips_and_reaches_the_policy(self):
    prefs = replace(decode_preferences(OLD), mode=ModeChoice.CCM,
                    ccm=replace(decode_preferences(OLD).ccm, climb=True, climb_lead=True))
    raw = encode_preferences(prefs)
    doc = json.loads(raw)
    self.assertEqual((doc['ccm']['climb'], doc['ccm']['climb_lead']), (True, True))
    self.assertNotIn('climb_on_bend_exit', doc['ccm'])
    again = decode_preferences(raw)
    self.assertEqual(again, prefs)
    settings = again.mode_settings()
    self.assertEqual((settings.ccm_climb, settings.ccm_climb_on_bend_exit, settings.ccm_climb_lead), (True, False, True))

  def test_unknown_or_wrongly_typed_options_are_still_rejected(self):
    doc = json.loads(OLD)
    doc['ccm']['climb'] = 1
    with self.assertRaises(PreferenceError):
      decode_preferences(json.dumps(doc))
    doc['ccm']['climb'] = True
    doc['ccm']['climb_sideways'] = True
    with self.assertRaises(PreferenceError):
      decode_preferences(json.dumps(doc))

  def test_defaults_are_off(self):
    self.assertEqual((CCMOptions().climb, CCMOptions().climb_on_bend_exit, CCMOptions().climb_lead), (False, False, False))
    self.assertFalse(SavedPreferences().mode_settings().ccm_climb)


if __name__ == '__main__':
  unittest.main()
