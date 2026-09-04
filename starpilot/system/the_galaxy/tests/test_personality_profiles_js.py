import json
import subprocess
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "assets/components/tools/personality_profiles.mjs"
DEVICE_SETTINGS_PATH = MODULE_PATH.with_name("device_settings.js")


def _run_node(script):
  harness = f"""
    import {{ PROFILE_CLIPBOARD_SCHEMA_VERSION, copyCurve, formatSpeedMph, pasteCurve, valueFromPointer }} from {json.dumps(MODULE_PATH.as_uri())};
    {script}
  """
  result = subprocess.run(["node", "--input-type=module"], input=harness, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr
  return json.loads(result.stdout)


def test_graph_copy_and_paste_are_versioned_typed_and_isolated():
  result = _run_node("""
    const source = [0.8, 0.9, 1.0];
    const clipboard = copyCurve("braking", source);
    source[0] = 1.8;
    const pasted = pasteCurve(clipboard, "braking", 3);
    pasted[1] = 1.9;
    console.log(JSON.stringify({ version: PROFILE_CLIPBOARD_SCHEMA_VERSION, clipboard, pasted }));
  """)
  assert result["version"] == 1
  assert result["clipboard"] == {"schemaVersion": 1, "category": "braking", "curve": [0.8, 0.9, 1.0]}
  assert result["pasted"] == [0.8, 1.9, 1.0]


def test_paste_rejects_wrong_schema_category_length_or_nonfinite_data():
  result = _run_node("""
    const good = copyCurve("acceleration", [1, 2, 3]);
    console.log(JSON.stringify([
      pasteCurve({ ...good, schemaVersion: 2 }, "acceleration", 3),
      pasteCurve(good, "braking", 3),
      pasteCurve(good, "acceleration", 2),
      pasteCurve({ ...good, curve: [1, null, 3] }, "acceleration", 3),
    ]));
  """)
  assert result == [None, None, None, None]


def test_graph_pointer_values_are_clamped_and_snapped():
  result = _run_node("""
    console.log(JSON.stringify([
      valueFromPointer(100, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
      valueFromPointer(200, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
      valueFromPointer(350, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
    ]));
  """)
  assert result == [2.0, 1.25, 0.5]


def test_speed_labels_keep_exact_axes_but_render_compact_values():
  result = _run_node("""
    console.log(JSON.stringify([formatSpeedMph(0), formatSpeedMph(11.184681), formatSpeedMph(90)]));
  """)
  assert result == ["0", "11.2", "90"]


def test_rendered_editor_has_parked_locks_units_and_all_three_profile_categories():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  assert 'disabled="${() => !!state.values.IsOnroad' in source
  assert 'aria-disabled="${() => !!state.values.IsOnroad}"' in source
  assert "Speed axis: mph · Value axis:" in source
  assert "m/s²" in source
  for category in ("acceleration", "braking", "following"):
    assert f'renderPersonalityCategoryField(profile, "{category}"' in source
  assert 'following: { label: "Following"' in source
  assert 'param.key === "CustomPersonalities" && state.expanded[param.key]' in source
  assert 'param.key === "CustomPersonalities" && isParamEnabledForChildren(param)' not in source
  assert '<button type="button" class="ds-manage-btn"' in source


def test_profile_errors_are_escaped_before_the_legacy_html_snackbar_sink():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  helper = source.split("function showParamSnackbar", 1)[1].split("}\n", 1)[0]
  assert "escapeSnackbarText(message)" in helper


def test_personality_cards_replace_legacy_follow_rows_without_changing_their_runtime_keys():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  advanced = source.split("const PERSONALITY_ADVANCED_KEYS = {", 1)[1].split("}\n", 1)[0]
  hidden = source.split("const HIDDEN_SETTING_KEYS = new Set([", 1)[1].split("]);", 1)[0]
  for key in (
    "TrafficFollow",
    "AggressiveFollow",
    "AggressiveFollowHigh",
    "StandardFollow",
    "StandardFollowHigh",
    "RelaxedFollow",
    "RelaxedFollowHigh",
  ):
    assert f'"{key}"' not in advanced
    assert f'"{key}"' in hidden


def test_traffic_card_restores_the_base_dom_traffic_mode_toggle():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  assert "function renderTrafficModeToggle" in source
  toggle = source.split("function renderTrafficModeToggle", 1)[1].split("\n}", 1)[0]
  assert "TrafficPersonalityProfile" in toggle
  assert 'aria-label="${param.label}"' in toggle
  assert 'updateParam("TrafficPersonalityProfile", "checkbox")' in toggle
  card = source.split("function renderPersonalityCardSnapshot", 1)[1].split("\n}", 1)[0]
  assert "renderTrafficModeToggle(profile)" in card


def test_traffic_mode_toggle_controls_traffic_editor_visibility():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  card = source.split("function renderPersonalityCardSnapshot", 1)[1].split("\n}", 1)[0]
  assert 'const settingsVisible = profile.id !== "traffic" || !!state.values.TrafficPersonalityProfile' in card
  assert '${settingsVisible ? html`' in card


def test_profile_preset_select_binds_the_selected_dom_property():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  field = source.split("function renderPersonalityCategoryField", 1)[1].split("\n}", 1)[0]
  assert ' selected="${() => config.preset === option}"' in field
  assert ' selected="${config.preset === option}"' not in field


def test_advanced_disclosure_uses_the_concise_advanced_label():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  advanced = source.split("function renderPersonalityAdvanced(profile)", 1)[1].split("\n}", 1)[0]
  assert "${isOpen ? \"Hide\" : \"Show\"} existing smoothness & response controls" not in advanced
  assert "\n        Advanced\n" in advanced


def test_advanced_disclosure_updates_in_place_without_rerendering_the_card():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  advanced = source.split("function renderPersonalityAdvanced(profile)", 1)[1].split("\n}", 1)[0]
  assert "const isOpen =" not in advanced
  assert 'aria-expanded="${() => state.personalityAdvancedExpanded[profile.id] ? "true" : "false"}"' in advanced
  assert "${renderPersonalityAdvancedRows(profile)}" in advanced
  rows = source.split("function renderPersonalityAdvancedRows(profile)", 1)[1].split("\n}", 1)[0]
  assert 'hidden="${() => !state.personalityAdvancedExpanded[profile.id]}"' in rows
  assert "PERSONALITY_ADVANCED_KEYS[profile.id]" in rows
  assert "renderSettingRow" in rows
