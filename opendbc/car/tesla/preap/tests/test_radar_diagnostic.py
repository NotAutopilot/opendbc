import pytest

from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.radar_diagnostic import (
  FLOW_CONTROL,
  RADAR_DIAGNOSTIC_BUS,
  RADAR_DIAGNOSTIC_RX_ADDRESS,
  RADAR_IDENTITY_DIDS,
  RadarDiagnosticFailure,
  RadarDiagnosticProbe,
  RadarDiagnosticState,
  RadarIdentityRecord,
)


def _response(payload: bytes) -> CanData:
  assert len(payload) <= 7
  return CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, bytes([len(payload)]) + payload + bytes(7 - len(payload)), RADAR_DIAGNOSTIC_BUS)


def _advance(probe: RadarDiagnosticProbe, payload: bytes, now: float):
  probe.update([], now)
  return probe.update([_response(payload)], now)


def _enter_read_did(probe: RadarDiagnosticProbe) -> None:
  probe.start(0.0)
  _advance(probe, b"\x7e\x00", 0.1)
  _advance(probe, b"\x50\x01", 0.2)
  _advance(probe, b"\x50\x03", 0.3)
  _advance(probe, b"\x7e\x00", 0.4)


def _enter_dtc_inventory(probe: RadarDiagnosticProbe) -> None:
  _enter_read_did(probe)
  for index, did in enumerate(RADAR_IDENTITY_DIDS, start=5):
    _advance(probe, b"\x62" + did.to_bytes(2, "big"), index / 10)


def _complete_cleanup(probe: RadarDiagnosticProbe, now: float):
  probe.update([], now)
  return probe.update([_response(b"\x50\x01")], now)


def test_probe_uses_only_the_curated_did_sequence():
  with pytest.raises(TypeError):
    RadarDiagnosticProbe((0xF190,))

  probe = RadarDiagnosticProbe()
  _enter_read_did(probe)
  sends = probe.update([], 0.5).can_sends
  assert sends[0].dat == b"\x03\x22\xa0\x22\x00\x00\x00\x00"
  assert 0xF190 not in probe.identifiers


def test_read_only_probe_collects_bounded_dtc_details_and_cleans_up():
  probe = RadarDiagnosticProbe()
  _enter_dtc_inventory(probe)
  _advance(probe, b"\x59\x02\xff\x12\x34\x56\x09", 2.4)
  _advance(probe, b"\x59\x04\x12\x34\x56S", 2.5)
  _advance(probe, b"\x59\x06\x12\x34\x56E", 2.6)
  output = _complete_cleanup(probe, 2.7)

  assert output.state == RadarDiagnosticState.COMPLETE
  assert output.report is not None
  assert output.report.records == tuple(RadarIdentityRecord(did, b"") for did in RADAR_IDENTITY_DIDS)
  assert output.report.dtc_details[0].snapshot == b"S"
  assert output.report.dtc_details[0].extended_data == b"E"
  assert output.report.cleanup_confirmed


def test_response_pending_and_multiframe_dtc_response_are_bounded():
  probe = RadarDiagnosticProbe()
  probe.start(0.0)
  probe.update([], 0.0)
  pending = probe.update([_response(b"\x7f\x3e\x78")], 0.1)
  assert pending.state == RadarDiagnosticState.TESTER_PRESENT
  assert not pending.can_sends
  assert probe.update([], 0.2).can_sends == ()
  _advance(probe, b"\x7e\x00", 0.3)

  _enter_dtc_inventory(probe)
  probe.update([], 2.4)
  payload = b"\x59\x02\xff\x12\x34\x56\x09\xab\xcd\xef\x08"
  first_frame = CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, b"\x10\x0b" + payload[:6], RADAR_DIAGNOSTIC_BUS)
  output = probe.update([first_frame], 2.5)
  assert [send.dat for send in output.can_sends] == [FLOW_CONTROL]
  output = probe.update([CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, b"\x21" + payload[6:] + bytes(7 - len(payload[6:])), RADAR_DIAGNOSTIC_BUS)], 2.6)
  assert output.state == RadarDiagnosticState.READ_DTC_SNAPSHOT
  assert not output.can_sends


def test_malformed_response_and_timeout_complete_fail_closed_cleanup():
  probe = RadarDiagnosticProbe()
  probe.start(0.0)
  probe.update([], 0.0)
  _advance(probe, b"\x7e\x00", 0.1)
  probe.update([], 0.2)
  output = probe.update([_response(b"\x50\x03")], 0.3)
  assert output.state == RadarDiagnosticState.CLEANUP
  output = _complete_cleanup(probe, 0.4)
  assert output.state == RadarDiagnosticState.FAILED
  assert output.report is not None and output.report.cleanup_confirmed

  probe = RadarDiagnosticProbe()
  probe.start(0.0)
  probe.update([], 0.0)
  _advance(probe, b"\x7e\x00", 0.1)
  probe.update([], 0.2)
  output = probe.update([], 3.3)
  assert output.state == RadarDiagnosticState.CLEANUP
  output = _complete_cleanup(probe, 3.4)
  assert output.state == RadarDiagnosticState.FAILED
  assert output.report is not None and output.report.failure == RadarDiagnosticFailure.TIMEOUT and output.report.cleanup_confirmed


def test_detail_response_limit_rejects_oversized_isotp_payload_then_cleans_up():
  probe = RadarDiagnosticProbe()
  _enter_dtc_inventory(probe)
  _advance(probe, b"\x59\x02\xff\x12\x34\x56\x09", 2.4)
  probe.update([], 2.5)
  first_frame = CanData(RADAR_DIAGNOSTIC_RX_ADDRESS, b"\x11\x01\x59\x04\x12\x34\x56\x00", RADAR_DIAGNOSTIC_BUS)
  output = probe.update([first_frame], 2.6)

  assert output.state == RadarDiagnosticState.CLEANUP
  assert not output.can_sends
  output = _complete_cleanup(probe, 2.7)
  assert output.state == RadarDiagnosticState.FAILED
  assert output.report is not None and output.report.cleanup_confirmed
