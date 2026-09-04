import json
import subprocess
from pathlib import Path

MODULE_PATH = Path(__file__).resolve().parents[1] / "assets/components/tools/personality_profiles.mjs"
DEVICE_SETTINGS_PATH = MODULE_PATH.with_name("device_settings.js")


def _run_node(script):
  harness = f"""
    import {{ copyCurve, pasteCurve, valueFromPointer }} from {json.dumps(MODULE_PATH.as_uri())};
    {script}
  """
  result = subprocess.run(["node", "--input-type=module"], input=harness, capture_output=True, text=True, timeout=30, check=False)
  assert result.returncode == 0, result.stderr
  return json.loads(result.stdout)


def test_graph_copy_and_paste_are_typed_and_do_not_share_mutable_arrays():
  result = _run_node("""
    const source = [0.8, 0.9, 1.0];
    const clipboard = copyCurve("braking", source);
    source[0] = 1.8;
    const pasted = pasteCurve(clipboard, "braking", 3);
    pasted[1] = 1.9;
    console.log(JSON.stringify({ clipboard, pasted, mismatch: pasteCurve(clipboard, "following", 3) }));
  """)

  assert result["clipboard"] == {"category": "braking", "curve": [0.8, 0.9, 1.0]}
  assert result["pasted"] == [0.8, 1.9, 1.0]
  assert result["mismatch"] is None


def test_graph_pointer_values_are_clamped_and_snapped():
  result = _run_node("""
    console.log(JSON.stringify([
      valueFromPointer(100, { top: 100, height: 200 }, 0.75, 3.0, 0.05),
      valueFromPointer(200, { top: 100, height: 200 }, 0.75, 3.0, 0.05),
      valueFromPointer(350, { top: 100, height: 200 }, 0.75, 3.0, 0.05),
    ]));
  """)

  assert result == [3.0, 1.9, 0.75]


def test_personality_ui_explains_drive_mode_mapping_precedence():
  source = DEVICE_SETTINGS_PATH.read_text(encoding="utf-8")

  assert "Vehicle drive-mode mapping always overrides personality acceleration and braking, including Traffic Mode." in source
