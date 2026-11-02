#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import sys
import usb1
import time
import signal
import subprocess

from panda import Panda, PandaDFU, PandaProtocolMismatch, McuType, FW_PATH
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.panda_wake import FirmwareChoice, firmware_path, startup_choice
from openpilot.starpilot.firmware_transaction import BarrierError, FirmwareBarrier
from openpilot.starpilot.firmware_ready import NativeStartup
from openpilot.starpilot.firmware_contract import file_sha256, serve_runtime, serve_blocked_runtime


DEFAULT_FIRMWARE_CHOICE = FirmwareChoice()
FIRMWARE_BARRIER_DIRECTORY = "/data/.starpilot-firmware-coordination"
FIRMWARE_BARRIER_OWNER_UID = 1000


def get_expected_signature(mcu_type: McuType, choice: FirmwareChoice = DEFAULT_FIRMWARE_CHOICE) -> bytes:
  fn = firmware_path(FW_PATH, mcu_type.config.app_fn, choice)
  return Panda.get_signature_from_firmware(fn)

def flash_panda(panda_serial: str, choice: FirmwareChoice = DEFAULT_FIRMWARE_CHOICE):
  panda = Panda(panda_serial)
  hardware = panda.get_type()
  tesla_cuatro = hardware == Panda.HW_TYPE_CUATRO and choice == FirmwareChoice(tesla=True)
  if choice.alternate and hardware not in (Panda.HW_TYPE_DOS, Panda.HW_TYPE_TRES, Panda.HW_TYPE_RED_PANDA) and not tesla_cuatro:
    choice = FirmwareChoice()
  fw_path = firmware_path(FW_PATH, panda.get_mcu_type().config.app_fn, choice)
  fw_signature = get_expected_signature(panda.get_mcu_type(), choice)
  internal_panda = panda.is_internal()

  panda_version = "bootstub" if panda.bootstub else panda.get_version()
  panda_signature = b"" if panda.bootstub else panda.get_signature()
  cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash(fn=fw_path)
    cloudlog.info("Done flashing")

  if panda.bootstub:
    bootstub_version = panda.get_version()
    cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
    if internal_panda:
      HARDWARE.recover_internal_panda()
    panda.recover(reset=(not internal_panda))
    cloudlog.info("Done flashing bootstub")

  if panda.bootstub:
    cloudlog.info("Panda still not booting, exiting")
    raise AssertionError

  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    cloudlog.info("Version mismatch after flashing, exiting")
    raise AssertionError

  if panda.get_packets_versions() != (Panda.HEALTH_PACKET_VERSION, Panda.CAN_PACKET_VERSION):
    panda.close()
    raise PandaProtocolMismatch("Panda CAN or health protocol hash mismatch after firmware validation")

  panda.close()
  return os.path.basename(fw_path)


def firmware_environment(selected_firmware: dict[str, str]) -> dict[str, str]:
  # Bind each flashed board's exact image to its hardware serial, including stock images.
  environment = {key: value for key, value in os.environ.items() if not key.startswith('STARPILOT_PANDA_FIRMWARE')}
  if not selected_firmware:
    raise ValueError('Missing Panda firmware selection')
  for serial, filename in selected_firmware.items():
    if len(serial) != 24 or any(char not in '0123456789abcdefABCDEF' for char in serial):
      raise ValueError('Invalid Panda hardware serial')
    environment['STARPILOT_PANDA_FIRMWARE_' + serial] = filename
  environment['STARPILOT_PANDA_FIRMWARE_MAP'] = '1'
  return environment


def wait_for_internal_panda(timeout: int = 15) -> bool:
  if HARDWARE.get_device_type() == "tici":
    # C3's internal USB port needs udev permissions in application and DFU modes.
    return subprocess.run(["udevadm", "wait", f"--timeout={timeout}", "/sys/bus/usb/devices/1-1.2"], check=False,
                          stderr=subprocess.DEVNULL if timeout == 0 else None).returncode == 0
  return True


def stop_unready_child(process):
  if process.poll() is not None:
    return
  process.send_signal(signal.SIGINT)
  try:
    process.wait(timeout=5)
  except subprocess.TimeoutExpired:
    process.kill()
    process.wait(timeout=5)


def loaded_contract_modules():
  return {
    "updater": sys.modules[__name__],
    "barrier": sys.modules["openpilot.starpilot.firmware_transaction"],
    "startup": sys.modules["openpilot.starpilot.firmware_ready"],
    "contract": sys.modules["openpilot.starpilot.firmware_contract"],
  }


def main() -> None:
  # signal pandad to close the relay and exit
  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    if process is not None:
      process.send_signal(signal.SIGINT)

  process = None
  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  # Provisioned by the separately qualified bootstrap, before any image changes.
  # Missing/untrusted coordination state inhibits the updater rather than bypassing it.
  barrier = FirmwareBarrier(FIRMWARE_BARRIER_DIRECTORY, owner_uid=FIRMWARE_BARRIER_OWNER_UID)
  heartbeat_checked = False
  count = 0
  while not do_exit:
    process = None
    try:
      blocked = False
      with barrier.reader() as lease:
        if not lease:
          blocked = True
        else:
          if not heartbeat_checked:
            # check health for lost heartbeat
            try:
              if wait_for_internal_panda(timeout=0):
                for s in Panda.list():
                  with Panda(s) as p:
                    health = p.health()
                    if p.is_internal() and health["heartbeat_lost"]:
                      Params().put_bool("PandaHeartbeatLost", True, block=True)
                      cloudlog.event("heartbeat lost", deviceState=health)
            except Exception:
              cloudlog.exception("pandad.uncaught_exception")
            heartbeat_checked = True
          cloudlog.event("pandad.flash_and_connect", count=count)
          if (count % 2) == 0:
            HARDWARE.reset_internal_panda()
          else:
            HARDWARE.recover_internal_panda()
          count += 1
          wait_for_internal_panda()

          # Flash all Pandas in DFU mode
          for serial in PandaDFU.list():
            cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
            PandaDFU(serial).recover()
            time.sleep(1)
            wait_for_internal_panda()

          panda_serials = Panda.list()
          if len(panda_serials):
            cloudlog.info(f"{len(panda_serials)} panda found, connecting - {panda_serials}")
            choice = startup_choice(Params())
            if len(set(panda_serials)) != len(panda_serials):
              raise ValueError('Duplicate Panda hardware serial')
            selected_firmware = {serial: flash_panda(serial, choice) for serial in panda_serials}
            # Capture identities while the same read lease still protects image reads.
            selected_images = {serial: {"filename": filename, "sha256": file_sha256(os.path.join(FW_PATH, filename))}
                               for serial, filename in selected_firmware.items()}
            ordering = []
            for serial in panda_serials:
              with Panda(serial) as panda:
                ordering.append((not panda.is_internal(), panda.get_type(), panda.get_usb_serial()))
            panda_serials = [serial for _, _, serial in sorted(ordering)]
            if set(panda_serials) != set(selected_firmware):
              raise ValueError('Panda hardware identity changed after flashing')

            # run real pandad
            os.environ['MANAGER_DAEMON'] = 'pandad'
            with NativeStartup(lease.descriptor) as startup:
              process = subprocess.Popen(["./pandad", *panda_serials], cwd=os.path.join(BASEDIR, "openpilot/selfdrive/pandad"),
                                         env=startup.environment(firmware_environment(selected_firmware)),
                                         pass_fds=startup.pass_fds)
              startup.child_started()
              try:
                startup.wait(process, lambda: do_exit)
              except BaseException:
                stop_unready_child(process)
                raise
      if blocked:
        # The reader scope has ended; do not block a recovery writer with SH.
        serve_blocked_runtime(barrier, loaded_contract_modules(), lambda: do_exit)
        continue
      if process is not None:
        try:
          # The endpoint exists only after private native ACK and read-lease exit.
          # It serves observations while the normal native child supplies telemetry.
          serve_runtime(process, loaded_contract_modules(), selected_images, lambda: do_exit)
        finally:
          # An observer/service failure must not leave an orphan native writer.
          stop_unready_child(process)
    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
    except (BarrierError, FileNotFoundError, PermissionError):
      cloudlog.exception("pandad.firmware_coordination_unavailable")
      time.sleep(1)
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")


if __name__ == "__main__":
  main()
