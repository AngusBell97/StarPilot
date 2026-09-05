import json
import subprocess
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "assets/components/tools/personality_profiles.mjs"
DEVICE_SETTINGS_PATH = MODULE_PATH.with_name("device_settings.js")
DEVICE_SETTINGS_CSS_PATH = MODULE_PATH.with_name("device_settings.css")
DEVICE_SETTINGS_LAYOUT_PATH = MODULE_PATH.parents[5] / "common/assets/device_settings_layout.json"


def _run_node(script):
  harness = f"""
    import {{ formatProfileSpeed, profileSpeedUnit, valueFromPointer }} from {json.dumps(MODULE_PATH.as_uri())};
    {script}
  """
  result = subprocess.run(["node", "--input-type=module"], input=harness, capture_output=True, text=True, timeout=30)
  assert result.returncode == 0, result.stderr
  return json.loads(result.stdout)


def test_graph_speed_labels_follow_the_selected_unit_system():
  result = _run_node("""
    console.log(JSON.stringify([
      formatProfileSpeed(10, false),
      formatProfileSpeed(10, true),
      profileSpeedUnit(false),
      profileSpeedUnit(true),
    ]));
  """)
  assert result == ["10", "16.1", "mph", "km/h"]


def test_graph_pointer_values_are_clamped_and_snapped():
  result = _run_node("""
    console.log(JSON.stringify([
      valueFromPointer(100, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
      valueFromPointer(200, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
      valueFromPointer(350, { top: 100, height: 200 }, 0.5, 2.0, 0.05),
    ]));
  """)
  assert result == [2.0, 1.25, 0.5]


def test_rendered_editor_has_parked_locks_units_and_all_three_profile_categories():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  assert 'disabled="${() => !!state.values.IsOnroad' in source
  assert 'aria-disabled="${() => !!state.values.IsOnroad || !!state.personalityMigrationRequired}"' in source
  assert "profileSpeedUnit" in source
  assert "m/s²" in source
  for category in ("acceleration", "braking", "following"):
    assert f'renderPersonalityCategoryField(profile, "{category}"' in source
  assert 'following: { label: "Following"' in source
  assert 'param.key === "CustomPersonalities" && state.expanded[param.key]' in source
  assert 'param.key === "CustomPersonalities" && isParamEnabledForChildren(param)' not in source
  assert '<button type="button" class="ds-manage-btn"' in source


def test_profile_master_and_advanced_controls_declare_parked_only_metadata():
  layout = json.loads(DEVICE_SETTINGS_LAYOUT_PATH.read_text(encoding="utf-8"))
  params = {param["key"]: param for section in layout for param in section.get("params", [])}
  keys = {
    "CustomPersonalities",
    *{f"{profile}PersonalityProfile" for profile in ("Traffic", "Aggressive", "Standard", "Relaxed")},
    "TrafficFollow",
    "AggressiveFollow",
    "AggressiveFollowHigh",
    "StandardFollow",
    "StandardFollowHigh",
    "RelaxedFollow",
    "RelaxedFollowHigh",
    *{
      f"{profile}{suffix}"
      for profile in ("Traffic", "Aggressive", "Standard", "Relaxed")
      for suffix in ("JerkAcceleration", "JerkDeceleration", "JerkDanger", "JerkSpeedDecrease", "JerkSpeed")
    },
  }
  assert all(params[key].get("requires_offroad") is True for key in keys)


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
  assert 'class="ds-personality-settings"' in card
  assert 'hidden="${() => profile.id === "traffic" && !state.values.TrafficPersonalityProfile}"' in card
  assert 'hidden="${() => profile.id !== "traffic" || !!state.values.TrafficPersonalityProfile}"' in card
  assert "settingsVisible ? html`" not in card


def test_profile_presets_are_direct_neutral_buttons_not_dropdowns():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  field = source.split("function renderPersonalityCategoryField", 1)[1].split("\n}", 1)[0]
  assert "<select" not in field
  assert 'aria-pressed="${() => config.preset === option ? "true" : "false"}"' in field
  assert "updatePersonalityPreset(profile.id, category, option)" in field


def test_personality_controls_have_visible_keyboard_focus_styles():
  css = DEVICE_SETTINGS_CSS_PATH.read_text(encoding="utf-8")
  for selector in (
    ".ds-personality-summary:focus-visible",
    ".ds-personality-option:focus-visible",
    ".ds-personality-advanced-choice:focus-visible",
    ".ds-personality-value input:focus-visible",
    ".ds-personality-custom-number input:focus-visible",
  ):
    assert selector in css


def test_custom_graph_has_reference_line_and_only_reset_action():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  curve = source.split("function renderPersonalityCurve", 1)[1].split("\n}", 1)[0]
  draw = source.split("function drawPersonalityCurve", 1)[1].split("\n}", 1)[0]
  assert "referenceCurve" in curve
  assert 'aria-label="${profile.label} ${definition.label} at ${formatProfileSpeed(geometry.speeds[index], !!state.values.IsMetric)} ${profileSpeedUnit(!!state.values.IsMetric)}"' in curve
  assert "referenceCurve" in draw
  assert "context.setLineDash([" in draw
  assert 'class="ds-personality-reference-key"' in curve
  assert "Dom default" in curve
  css = DEVICE_SETTINGS_CSS_PATH.read_text(encoding="utf-8")
  assert ".ds-personality-reference-key" in css
  assert "resetPersonalityCurve" in curve
  assert ">Reset<" in curve
  assert ">Copy<" not in curve
  assert ">Paste<" not in curve


def test_advanced_values_use_supported_presets_and_warn_before_custom():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  advanced_rows = source.split("function renderPersonalityAdvancedRows", 1)[1].split("\n}", 1)[0]
  value_editor = source.split("function renderPersonalityAdvancedValue", 1)[1].split("\n}", 1)[0]
  advanced = advanced_rows + value_editor
  assert "Custom values are untested" in advanced
  assert "Chill" in advanced
  assert "Standard" in advanced
  assert "Custom" in advanced
  assert "renderSettingRow" not in advanced
  assert "updatePersonalityAdvancedPreset" in source
  assert "ds-personality-advanced-choice" in value_editor
  assert 'min="${bounds.min}"' in value_editor
  assert 'max="${bounds.max}"' in value_editor
  assert 'step="${bounds.step}"' in value_editor
  assert "resolveCurrentNumericValue(param, bounds)" in value_editor


def test_profile_descriptions_are_removed():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  assert "Stop-and-go driving" not in source
  assert "Assertive driving with tighter gaps" not in source
  assert "Balanced everyday driving" not in source
  assert "Smoother driving with larger gaps" not in source


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
  assert "renderPersonalityAdvancedValue" in rows
  assert "renderSettingRow" not in rows


def test_profiles_panel_omits_the_redundant_enabled_intro_and_toggle():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  panel = source.split("function renderPersonalityProfilesPanel()", 1)[1].split("\n}", 1)[0]
  assert "ds-personality-intro" not in panel
  assert "Use per-personality longitudinal profiles" not in panel
  assert "Acceleration, cruise/SLC braking" not in panel


def test_dom_default_is_not_offered_in_profile_selectors():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  field = source.split("function renderPersonalityCategoryField", 1)[1].split("\n}", 1)[0]
  assert '.filter(option => option !== "dom_default")' in field


def test_schema_migration_state_is_visible_and_blocks_profile_writes():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  assert "personalityMigrationRequired: false" in source
  assert "state.personalityMigrationRequired = !!data.migration_required" in source
  assert "This profile data requires a verified migration before it can be edited." in source
  assert "!!state.personalityMigrationRequired" in source
  assert 'param?.key === "CustomPersonalities" && state.personalityMigrationRequired' in source
  css = DEVICE_SETTINGS_CSS_PATH.read_text(encoding="utf-8")
  assert ".ds-personality-migration-warning" in css


def test_all_personality_cards_start_collapsed():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  state_block = source.split("const state = reactive({", 1)[1].split("})", 1)[0]
  assert "personalityExpanded: {}," in state_block
  assert "personalityExpanded: { traffic: true }" not in state_block


def test_advanced_rows_are_hidden_by_author_css_when_collapsed():
  css = DEVICE_SETTINGS_CSS_PATH.read_text(encoding="utf-8")
  assert ".ds-personality-advanced-rows[hidden]" in css
  hidden_rule = css.split(".ds-personality-advanced-rows[hidden]", 1)[1].split("}", 1)[0]
  assert "display: none;" in hidden_rule


def test_personality_cards_keep_distinct_symbols_but_selectors_are_not_profile_coloured():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")
  css = DEVICE_SETTINGS_CSS_PATH.read_text(encoding="utf-8")
  for icon in ("bi-stoplights-fill", "bi-lightning-charge-fill", "bi-speedometer2", "bi-feather"):
    assert icon in source
  assert '<i class="${profile.icon}" aria-hidden="true"></i>' in source
  assert ".ds-personality-option[aria-pressed=\"true\"]" in css
  assert ".ds-personality-option[data-profile=" not in css
