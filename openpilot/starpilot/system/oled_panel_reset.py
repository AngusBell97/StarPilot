#!/usr/bin/env python3
"""comma four (Bantian RM67162 OLED): clear the red glow in black areas with one panel sleep/wake cycle.

After a (warm) boot the panel is handed over from the boot splash without ever going through a real Sleep In,
and black pixels glow red until it does. Recipe from kytos22's report on commaai/openpilot#38707: display off
+ sleep in, replay comma's stock panel init table (dsi-panel-bantian.dtsi), sleep out + display on. Nothing
persistent is changed and a reboot undoes it; it survives the normal screen off/on.

Run as root from launch_chffrplus.sh before the manager starts (the screen shows the boot logo then), once per
boot. The kernel's debugfs mipi_command ignores the delay byte in each command, so the delays are done here, and
the panel's power mode (DCS 0x0A) is read back to confirm each step: 0x9C = awake and on, 0x08 = asleep.

Usage: oled_panel_reset.py [--boot] [--dry-run] [--dark-ms 450]
"""
import argparse
import os
import struct
import subprocess
import time

CMD = "/sys/kernel/debug/dsi_dwo_video_display/mipi_command"
INFO = "/sys/kernel/debug/dsi_dwo_video_display/dump_info"
BL = "/sys/class/backlight/panel0-backlight"
ONROAD = "/data/params/d/IsOnroad"
DONE = "/tmp/oled_panel_reset.done"  # /tmp is cleared by a reboot: once per boot
LOG = "/data/oled-black/reset.log"
AWAKE = 0x9C

DISPOFF = "05 01 00 00 00 00 02 28 00"
SLPIN = "05 01 00 00 78 00 02 10 00"
SLPOUT = "05 01 00 00 64 00 02 11 00"
DISPON = "05 01 00 00 78 00 02 29 00"
# comma's stock Bantian on-command table up to SLPOUT. Its final brightness command (51 D0 D0) is left out: the
# backlight is restored from sysfs afterwards so the current brightness is kept.
INIT = [
  "39 01 00 00 0A 00 02 FE 05",
  "39 01 00 00 0A 00 02 91 00",
  "39 01 00 00 0A 00 02 00 00",
  "39 01 00 00 0A 00 02 FE 01",
  "39 01 00 00 0A 00 02 1C 99",
  "39 01 00 00 0A 00 02 FE 00",
  "39 01 00 00 0A 00 02 3A 77",
  "39 01 00 00 0A 00 02 51 00",
  "39 01 00 00 0A 00 02 36 00",
  "39 01 00 00 0A 00 02 35 01",
  "39 01 00 00 0A 00 02 C2 03",
]


def pack(cmd: str) -> bytes:
  """One command for mipi_command: int length, then the bytes (same format as /usr/comma/screen_calibration.py)."""
  b = bytes(int(x, 16) for x in cmd.split())
  return struct.pack(f"i{len(b)}s", len(b), b)


class Panel:
  def __init__(self, dry: bool):
    self.dry = dry
    self.sent: list[str] = []

  def send(self, cmd: str) -> None:
    self.sent.append(cmd)
    if not self.dry:
      with open(CMD, "wb") as f:
        f.write(pack(cmd))

  def power_mode(self) -> int | None:
    if self.dry:
      return AWAKE
    try:
      with open(CMD, "wb") as f:
        f.write(pack("06 01 00 00 00 00 01 0A 00 01"))  # DCS read 0x0A, 1 byte back
      with open(CMD, "rb") as f:
        v = f.read(1)
      return v[0] if v else None
    except OSError:
      return None


def hexs(v: int | None) -> str:
  return "none" if v is None else f"0x{v:02X}"


def read(path: str) -> str:
  try:
    with open(path) as f:
      return f.read().strip()
  except OSError:
    return ""


def log(msg: str) -> None:
  line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}"
  print(line)
  try:
    os.makedirs(os.path.dirname(LOG), exist_ok=True)
    lines = []
    if os.path.exists(LOG):
      with open(LOG) as f:
        lines = f.read().splitlines()[-200:]
    with open(LOG, "w") as f:
      f.write("\n".join(lines + [line]) + "\n")
  except OSError:
    pass


def cycle(p: Panel, dark_ms: int) -> tuple[int | None, int | None]:
  """Sleep and wake the panel. Returns the power mode while asleep and after waking."""
  p.send(DISPOFF); time.sleep(0.03)
  p.send(SLPIN); time.sleep(0.12)
  asleep = p.power_mode()
  time.sleep(dark_ms / 1000)
  for c in INIT:
    p.send(c); time.sleep(0.01)
  awake = None
  for _ in range(3):  # wake, and retry the wake if the panel does not report back on
    p.send(SLPOUT); time.sleep(0.12)
    p.send(DISPON); time.sleep(0.12)
    awake = p.power_mode()
    if awake == AWAKE:
      break
  return asleep, awake


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("--boot", action="store_true", help="start-up mode: once per boot, quiet skips")
  ap.add_argument("--dry-run", action="store_true")
  ap.add_argument("--dark-ms", type=int, default=450, help="extra time asleep so the panel discharges")
  a = ap.parse_args()

  if a.boot and os.path.exists(DONE):
    return 0
  if not os.path.exists(CMD) or "bantian" not in read(INFO):
    log("skipped: not a Bantian panel or no panel command file")
    return 0
  if read(ONROAD) not in ("", "0"):
    log("skipped: onroad")
    return 0

  p = Panel(a.dry_run)
  before = p.power_mode()
  if before != AWAKE:
    log(f"skipped: panel not awake and on (power mode {hexs(before)})")
    return 0
  brightness = read(f"{BL}/brightness")
  asleep, awake = cycle(p, a.dark_ms)
  if not a.dry_run:
    if brightness:
      with open(f"{BL}/brightness", "w") as f:  # puts the brightness back (the init table set it to 0)
        f.write(brightness)
    # Same as boot: reapply comma's saved gamma (the cycle keeps it; this also covers the boot order).
    if os.path.exists("/usr/comma/screen_calibration.py"):
      subprocess.run(["python3", "/usr/comma/screen_calibration.py"], capture_output=True, timeout=10)
    with open(DONE, "w") as f:
      f.write("1\n")
  ok = awake == AWAKE
  log(f"{'ok' if ok else 'PANEL DID NOT WAKE'}: power mode {hexs(before)} -> {hexs(asleep)} -> {hexs(awake)}, "
      f"brightness {brightness}{' (dry run)' if a.dry_run else ''}")
  return 0 if ok else 1


if __name__ == "__main__":
  raise SystemExit(main())
