from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.radar_diagnostic import (
  FLOW_CONTROL,
  RADAR_DIAGNOSTIC_BUS,
  RADAR_DIAGNOSTIC_RX_ADDRESS,
  RadarDiagnosticProbe,
  RadarDiagnosticState,
)


def _response(payload: bytes) -> CanData:
  return CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, bytes([len(payload)]) + payload + bytes(7 - len(payload)), RADAR_DIAGNOSTIC_BUS)


def _advance(probe: RadarDiagnosticProbe, payload: bytes, now: float):
  probe.update([], now)
  return probe.update([_response(payload)], now)


def test_read_only_probe_collects_bounded_dtc_details_and_cleans_up():
  probe = RadarDiagnosticProbe((0xF180,))
  probe.start(0.0)
  assert probe.update([], 0.0).can_sends[0].dat == b"\x02\x3e\x00" + bytes(5)
  _advance(probe, b"\x7e\x00", 0.1)
  _advance(probe, b"\x50\x01", 0.2)
  _advance(probe, b"\x50\x03", 0.3)
  _advance(probe, b"\x7e\x00", 0.4)
  _advance(probe, b"\x62\xf1\x80ID", 0.5)
  _advance(probe, b"\x59\x02\xff\x12\x34\x56\x09", 0.6)
  _advance(probe, b"\x59\x04\x12\x34\x56S", 0.7)
  _advance(probe, b"\x59\x06\x12\x34\x56E", 0.8)
  output = _advance(probe, b"\x50\x01", 0.9)

  assert output.state == RadarDiagnosticState.COMPLETE
  assert output.report is not None
  assert output.report.records[0].value == b"ID"
  assert output.report.dtc_details[0].snapshot == b"S"
  assert output.report.dtc_details[0].extended_data == b"E"
  assert output.report.cleanup_confirmed


def test_detail_response_limit_rejects_oversized_isotp_payload():
  probe = RadarDiagnosticProbe(())
  probe.start(0.0)
  _advance(probe, b"\x7e\x00", 0.1)
  _advance(probe, b"\x50\x01", 0.2)
  _advance(probe, b"\x50\x03", 0.3)
  _advance(probe, b"\x7e\x00", 0.4)
  _advance(probe, b"\x59\x02\xff\x12\x34\x56\x09", 0.5)
  probe.update([], 0.6)
  first_frame = CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, b"\x11\x01\x59\x04\x12\x34\x56\x00", RADAR_DIAGNOSTIC_BUS)
  output = probe.update([first_frame], 0.7)

  assert output.state == RadarDiagnosticState.CLEANUP
  assert not output.can_sends
  assert FLOW_CONTROL not in [send.dat for send in output.can_sends]
