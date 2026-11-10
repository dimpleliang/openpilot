"""
dimpleliang/ars510: brake probe (11/10). Logs what the car's own computers do with openpilot's brake request, to
work out why the same request gives 0.14 or 0.34 m/s^2 at the stop and what the driver's own stop looks like in
brake pressure. Nothing here is used for control.

While the flag file exists, a few data identifiers are read over UDS (ReadDataByIdentifier 0x22, default session,
read-only) from the brake ECU (0x7B0 -> 0x7B8) and the hybrid / cruise ECU (0x7D2 -> 0x7DA) on bus 0, each at 10 Hz.
The answers are ordinary CAN frames and land in the rlog (can, bus 0); tools/ars510/mac/analysis/brake_probe.py decodes
them. Identifiers and scaling from 4piu/toyota-telemetry-XA50 signals.csv, verified on the car in park on 11/10
(/data/ars510_captures/didscan_2026-11-10.log): every one answers within 5-15 ms and moves with the brake pedal.

  0x7B0  0x1022  FR / FL / RR / RL target oil pressure, raw x 4 / 51 MPa            (single frame)
         0x1004  master cylinder pressure, raw x 0.098 - 1 MPa
         0x104C  brake pedal stroke, mm
         0x1008  forward / rearward G (byte 1), raw x 0.196 m/s^2
         0x1055  front regenerative operation, raw x 16 Nm
  0x7D2  0x15ED  cruise request throttle, driving support ECU target acceleration, cruise target acceleration,
                 cruise request driving force, feedback status                      (multi frame: 11 bytes)
         0x15EC  vehicle acceleration, gradient of road surface, output driving force, operation condition, gear (8)
         0x10A1  target motor torque                                                (9)
         0x1067  master cylinder control torque, raw x 0.125 - 4096 Nm              (single frame)
         0x15FE  cruise brake control decision / permission bits                    (single frame)

A multi-frame answer needs an ISO-TP flow control frame from the tester after the ECU's first frame. It is sent blind
one control frame (10 ms) after the request: the first frame arrives 5-8 ms after the request. If the flow control
is early the ECU drops that answer and the next poll gets it.

panda (safety/modes/toyota.h) lets through only: 03 22 10|15 xx 00 00 00 00 and 30 00 00 00 00 00 00 00 on these
two addresses. On: touch /data/ars510_brake_probe (checked once a second, no restart). Off: remove the file.
"""
import os

from opendbc.car.can_definitions import CanData

FLAG_FILE = "/data/ars510_brake_probe"
CHECK_PERIOD = 100    # frames (1 s) between flag file checks
START_FRAME = 500     # let the car settle (and fingerprinting finish) before the first request
CYCLE = 10            # frames: every identifier at 10 Hz
BRAKE_ECU = 0x7B0
HYBRID_ECU = 0x7D2
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"

# slot in the cycle -> (address, identifier or None for the flow control)
SCHEDULE = {
  0: [(BRAKE_ECU, 0x1022), (HYBRID_ECU, 0x15ED)],
  1: [(HYBRID_ECU, None)],
  2: [(BRAKE_ECU, 0x1004)],
  3: [(HYBRID_ECU, 0x15EC)],
  4: [(HYBRID_ECU, None), (BRAKE_ECU, 0x104C)],
  6: [(BRAKE_ECU, 0x1008), (HYBRID_ECU, 0x10A1)],
  7: [(HYBRID_ECU, None)],
  8: [(BRAKE_ECU, 0x1055), (HYBRID_ECU, 0x1067)],
  9: [(HYBRID_ECU, 0x15FE)],
}


def read_request(addr: int, did: int) -> CanData:
  return CanData(addr, bytes([0x03, 0x22, did >> 8, did & 0xFF, 0, 0, 0, 0]), 0)


class BrakeProbe:
  def __init__(self):
    self.enabled = False

  def update(self, frame: int) -> list[CanData]:
    if frame % CHECK_PERIOD == 0:
      self.enabled = os.path.exists(FLAG_FILE)
    if not self.enabled or frame < START_FRAME:
      return []
    return [CanData(addr, FLOW_CONTROL, 0) if did is None else read_request(addr, did)
            for addr, did in SCHEDULE.get(frame % CYCLE, [])]
