import math
from pathlib import Path
from typing import cast

from openpilot.selfdrive.ui.onroad.starpilot.developer_metrics import build_developer_metric_parts


ONROAD_VIEW = Path("selfdrive/ui/onroad/starpilot/starpilot_onroad_view.py")


def test_comma4_developer_metrics_use_published_temperatures_and_real_memory_capacity():
  parts = build_developer_metric_parts(
    show_fps=True,
    show_cpu=True,
    show_gpu=True,
    show_temp=True,
    show_memory=True,
    fps=20.4,
    min_fps=18.2,
    max_fps=21.1,
    avg_fps=19.8,
    cpu_usage_percent=[20, 24, 2, 44, 0, 0, 0, 0],
    cpu_temp_c=[58.4, 58.4, 58.8, 59.4, 57.8, 58.1, 57.8, 58.1],
    gpu_usage_percent=0,
    gpu_temp_c=[57.1, 57.8],
    max_temp_c=59.5,
    memory_usage_percent=32,
    memory_total_gib=3.52,
  )

  assert parts == [
    "CPU: 11% / 59°C",
    "GPU: 0% / 58°C",
    "TEMP: 60°C",
    "RAM: 1.1/3.5 GiB (32%)",
    "FPS: 20",
    "Min: 18",
    "Max: 21",
    "Avg: 20",
  ]


def test_onroad_view_uses_the_device_agnostic_metric_formatter():
  source = ONROAD_VIEW.read_text()

  assert "self._memory_total_gib = system_memory_total_gib()" in source
  assert "cpu_temps = list(device_state.cpuTempC)" in source
  assert "gpu_temps = list(device_state.gpuTempC)" in source
  assert "parts = build_developer_metric_parts(" in source
  assert "if math.isfinite(fps) and fps > 0:" in source
  assert "int(device_state.gpuUsagePercent)" not in source
  assert "int(device_state.memoryUsagePercent)" not in source
  assert "float(device_state.maxTempC)" not in source
  assert "mem_gb = 8.0 * mem_val / 100.0" not in source


def test_nonfinite_usage_values_fall_back_without_crashing_the_ui():
  parts = build_developer_metric_parts(
    show_fps=True,
    show_cpu=True,
    show_gpu=True,
    show_temp=True,
    show_memory=True,
    fps=math.inf,
    min_fps=-math.inf,
    max_fps=math.nan,
    avg_fps=math.inf,
    cpu_usage_percent=cast(list[int], [math.inf, -math.inf, math.nan]),
    cpu_temp_c=[],
    gpu_usage_percent=cast(int, math.inf),
    gpu_temp_c=[],
    max_temp_c=math.nan,
    memory_usage_percent=cast(int, math.nan),
    memory_total_gib=math.inf,
  )

  assert parts == [
    "CPU: 0%",
    "GPU: 0%",
    "TEMP: --",
    "RAM: 0%",
    "FPS: --",
    "Min: --",
    "Max: --",
    "Avg: --",
  ]
