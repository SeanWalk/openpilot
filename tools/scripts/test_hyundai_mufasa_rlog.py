#!/usr/bin/env python3
"""Check Hyundai CAN-FD decoding and firmware fingerprinting on a local rlog."""

import argparse

from opendbc.can import CANParser
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.hyundai.values import CAR
from openpilot.tools.lib.logreader import LogReader, ReadMode


def main(path: str) -> None:
  dbc = "hyundai_canfd_generated"
  parser = CANParser(dbc, [], 1)
  # Register the two messages whose values are checked below.
  parser.vl["WHEEL_SPEEDS"]
  parser.vl["STEERING_SENSORS"]

  frames_seen = {"WHEEL_SPEEDS": 0, "STEERING_SENSORS": 0}
  max_wheel_speeds = dict.fromkeys(("WHL_SpdFLVal", "WHL_SpdFRVal", "WHL_SpdRLVal", "WHL_SpdRRVal"), 0.0)
  steering_angle_range = [float("inf"), float("-inf")]
  log_fw = []
  logged_candidate = "unknown"
  for msg in LogReader(path, default_mode=ReadMode.RLOG):
    if msg.which() == "carParams":
      log_fw = list(msg.carParams.carFw)
      logged_candidate = msg.carParams.carFingerprint
    if msg.which() != "can":
      continue
    frames = [(can.address, bytes(can.dat), can.src) for can in msg.can]
    for address, _, src in frames:
      if src == 1 and address == 0xA0:
        frames_seen["WHEEL_SPEEDS"] += 1
      elif src == 1 and address == 0x125:
        frames_seen["STEERING_SENSORS"] += 1
    updated = parser.update((msg.logMonoTime, frames))
    if 0xA0 in updated:
      for signal in max_wheel_speeds:
        max_wheel_speeds[signal] = max(max_wheel_speeds[signal], parser.vl["WHEEL_SPEEDS"][signal])
    if 0x125 in updated:
      angle = parser.vl["STEERING_SENSORS"]["STEERING_ANGLE"]
      steering_angle_range[0] = min(steering_angle_range[0], angle)
      steering_angle_range[1] = max(steering_angle_range[1], angle)

  wheels = parser.vl["WHEEL_SPEEDS"]
  steering = parser.vl["STEERING_SENSORS"]
  assert all(frames_seen.values()), f"Missing required frames: {frames_seen}"
  assert any(wheels[s] != 0 for s in ("WHL_SpdFLVal", "WHL_SpdFRVal", "WHL_SpdRLVal", "WHL_SpdRRVal")), \
    f"Wheel speeds did not decode to non-zero values: {wheels}"
  assert "STEERING_ANGLE" in steering and "STEERING_RATE" in steering

  _, candidates = match_fw_to_car(log_fw, "", log=False)
  assert candidates == {CAR.HYUNDAI_MUFASA}, f"Unexpected firmware fingerprint result: {candidates}"

  print(f"DBC: {dbc}")
  print(f"Parsed frames: {frames_seen}")
  print("Wheel speed (km/h): " + ", ".join(f"{s}={wheels[s]:.2f}" for s in
    ("WHL_SpdFLVal", "WHL_SpdFRVal", "WHL_SpdRLVal", "WHL_SpdRRVal")))
  print("Maximum wheel speed (km/h): " + ", ".join(f"{s}={v:.2f}" for s, v in max_wheel_speeds.items()))
  print(f"Steering: angle={steering['STEERING_ANGLE']:.1f} deg (range {steering_angle_range[0]:.1f}..{steering_angle_range[1]:.1f}), " +
        f"rate={steering['STEERING_RATE']:.1f} deg/s")
  print(f"Car fingerprint: recorded={logged_candidate}, firmware match={next(iter(candidates))}")


if __name__ == "__main__":
  arg_parser = argparse.ArgumentParser(description=__doc__)
  arg_parser.add_argument("rlog", help="Path to an rlog.zst file")
  main(arg_parser.parse_args().rlog)
