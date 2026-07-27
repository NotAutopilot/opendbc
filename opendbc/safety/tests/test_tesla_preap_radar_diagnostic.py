import unittest

from opendbc.car.can_definitions import CanData
from opendbc.car.structs import CarParams
from opendbc.car.tesla.preap.radar_diagnostic import RadarDiagnosticProbe, RadarDiagnosticState
from opendbc.car.tesla.preap.safety_flags import TeslaPreAPSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py


TX_ADDR = 0x641
RX_ADDR = 0x651
BUS = 1
TESTER_PRESENT = b"\x02\x3e\x00\x00\x00\x00\x00\x00"
DEFAULT_SESSION = b"\x02\x10\x01\x00\x00\x00\x00\x00"
EXTENDED_SESSION = b"\x02\x10\x03\x00\x00\x00\x00\x00"
READ_DTCS = b"\x03\x19\x02\xff\x00\x00\x00\x00"
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"
DIDS = (0xA022, 0xF014, 0xF015, 0xF180, 0xF181, 0xF182, 0xF187, 0xF188, 0xF189, 0xF18A, 0xF18C, 0xF191, 0xF192, 0xF193, 0xF194, 0xF195, 0xF197, 0xF19E)


def _did_request(did: int) -> bytes:
  return b"\x03\x22" + did.to_bytes(2, "big") + bytes(4)


class TestTeslaPreAPRadarDiagnostic(unittest.TestCase):
  TX_MSGS = [[TX_ADDR, BUS]]

  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(TeslaPreAPSafetyFlags.RADAR_DIAGNOSTIC))
    self.safety.init_tests()

  def tx(self, data: bytes, addr: int = TX_ADDR, bus: int = BUS) -> bool:
    return self.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def rx(self, data: bytes, addr: int = RX_ADDR, bus: int = BUS) -> None:
    self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def single(self, payload: bytes) -> None:
    assert len(payload) <= 7
    self.rx(bytes([len(payload)]) + payload + bytes(7 - len(payload)))

  def request(self, data: bytes, response: bytes) -> None:
    assert self.tx(data)
    self.single(response)

  def enter_dtc_inventory(self) -> None:
    self.request(TESTER_PRESENT, b"\x7e\x00")
    self.request(DEFAULT_SESSION, b"\x50\x01")
    self.request(EXTENDED_SESSION, b"\x50\x03")
    self.request(TESTER_PRESENT, b"\x7e\x00")
    for did in DIDS:
      self.request(_did_request(did), b"\x62" + did.to_bytes(2, "big"))

  def enter_read_did(self) -> None:
    self.request(TESTER_PRESENT, b"\x7e\x00")
    self.request(DEFAULT_SESSION, b"\x50\x01")
    self.request(EXTENDED_SESSION, b"\x50\x03")
    self.request(TESTER_PRESENT, b"\x7e\x00")

  def cleanup(self) -> None:
    assert self.tx(DEFAULT_SESSION)
    self.single(b"\x50\x01")

  def recover(self) -> None:
    self.cleanup()
    self.cleanup()

  def advance_recovery(self, probe: RadarDiagnosticProbe, now: float):
    self.safety.set_timer(round(now * 1_000_000))
    output = probe.update([], now)
    assert len(output.can_sends) == 1
    request = output.can_sends[0]
    assert self.tx(request.dat, addr=request.address, bus=request.src)
    response = b"\x02\x50\x01\x00\x00\x00\x00\x00"
    self.rx(response)
    return probe.update([CanData(RX_ADDR, response, BUS)], now)

  def multiframe(self, payload: bytes) -> None:
    self.rx(bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6])
    assert self.tx(FLOW_CONTROL)
    offset, sequence = 6, 1
    while offset < len(payload):
      chunk = payload[offset : offset + 7]
      self.rx(bytes([0x20 | sequence]) + chunk + bytes(7 - len(chunk)))
      offset += len(chunk)
      sequence = (sequence + 1) & 0xF

  def test_forbidden_services_and_wrong_transport_are_rejected(self):
    assert not self.tx(b"\x02\x27\x11" + bytes(5))
    assert not self.tx(TESTER_PRESENT, addr=TX_ADDR + 1)
    assert not self.tx(TESTER_PRESENT, bus=0)
    assert not self.tx(TESTER_PRESENT[:-1])
    for request in (
      _did_request(0xF190),
      _did_request(0xA023),
      b"\x04\x31\x01\x0a\x03\x00\x00\x00",
      b"\x02\x11\x01\x00\x00\x00\x00\x00",
      b"\x01\x14\xff\x00\x00\x00\x00\x00",
      b"\x03\x2e\xf1\x90\x00\x00\x00\x00",
      b"\x03\x23\x00\x00\x00\x00\x00\x00",
      b"\x03\x34\x00\x00\x00\x00\x00\x00",
      b"\x03\x36\x00\x00\x00\x00\x00\x00",
      b"\x03\x37\x00\x00\x00\x00\x00\x00",
    ):
      with self.subTest(request=request):
        self.setUp()
        self.enter_read_did()
        assert not self.tx(request)
        self.recover()

  def test_inventory_binds_and_bounds_detail_requests(self):
    self.enter_dtc_inventory()
    codes = [index.to_bytes(3, "big") for index in range(1, 18)]
    assert self.tx(READ_DTCS)
    self.multiframe(b"\x59\x02\xff" + b"".join(code + b"\x09" for code in codes))
    for code in codes[:16]:
      self.request(b"\x06\x19\x04" + code + b"\xff\x00", b"\x59\x04" + code + b"\x01")
      self.request(b"\x06\x19\x06" + code + b"\xff\x00", b"\x59\x06" + code + b"\x02")
    assert not self.tx(b"\x06\x19\x04" + codes[16] + b"\xff\x00")
    assert self.tx(DEFAULT_SESSION)

  def test_valid_isotp_flow_control_is_one_shot(self):
    self.enter_dtc_inventory()
    payload = b"\x59\x02\xff\x12\x34\x56\x09\xab\xcd\xef\x08"
    assert self.tx(READ_DTCS)
    self.rx(b"\x10\x0b" + payload[:6])
    assert self.tx(FLOW_CONTROL)
    assert not self.tx(FLOW_CONTROL)
    self.recover()

  def test_malformed_response_timeout_and_latched_controls_fail_closed(self):
    assert self.tx(TESTER_PRESENT)
    assert not self.tx(b"\x40\x00\x00\x00", addr=0x488, bus=0)
    self.single(b"\x7e\x01")
    self.recover()
    assert not self.tx(TESTER_PRESENT)

    self.setUp()
    assert self.tx(TESTER_PRESENT)
    self.safety.set_timer(30_000_000)
    self.recover()
    assert not self.tx(TESTER_PRESENT)

  def test_cleanup_releases_controls_and_consumes_the_attempt(self):
    self.enter_dtc_inventory()
    self.request(READ_DTCS, b"\x59\x02\xff")
    self.request(DEFAULT_SESSION, b"\x50\x01")
    self.safety.set_controls_allowed(True)
    assert self.tx(b"\x40\x00\x00\x00", addr=0x488, bus=0)
    assert not self.tx(TESTER_PRESENT)

  def test_failed_normal_cleanup_requires_two_ack_recovery(self):
    for failure in ("malformed", "timeout"):
      with self.subTest(failure=failure):
        self.setUp()
        self.enter_dtc_inventory()
        self.request(READ_DTCS, b"\x59\x02\xff")
        assert self.tx(DEFAULT_SESSION)
        if failure == "malformed":
          self.single(b"\x7e\x00")
        else:
          self.safety.set_timer(3_000_000)

        self.recover()
        assert not self.tx(TESTER_PRESENT)

  def test_duplicate_cleanup_is_rejected_while_response_is_outstanding(self):
    assert self.tx(DEFAULT_SESSION)
    self.safety.set_timer(2_999_999)
    assert not self.tx(DEFAULT_SESSION)
    self.recover()

  def test_cleanup_replay_is_restart_idempotent_after_acknowledged_prefixes(self):
    for restart_prefixes in ((1,), (2,), (1, 1)):
      for reinitialize_panda in (False, True):
        with self.subTest(restart_prefixes=restart_prefixes, reinitialize_panda=reinitialize_panda):
          self.setUp()
          probe = RadarDiagnosticProbe()
          probe.start_cleanup(0.0)
          started_at = 0.0
          for acknowledged_before_restart in restart_prefixes:
            now = started_at + probe.RECOVERY_DRAIN_TIMEOUT
            assert probe.update([], now - 0.01).can_sends == ()
            for _ in range(acknowledged_before_restart):
              self.advance_recovery(probe, now)
              now += 0.1
            if reinitialize_panda:
              self.setUp()
            probe = RadarDiagnosticProbe()
            probe.start_cleanup(now)
            started_at = now

          now = started_at + probe.RECOVERY_DRAIN_TIMEOUT
          assert probe.update([], now - 0.01).can_sends == ()
          assert self.advance_recovery(probe, now).state == RadarDiagnosticState.CLEANUP
          output = self.advance_recovery(probe, now + 0.1)
          assert output.state == RadarDiagnosticState.COMPLETE
          assert output.report is not None and output.report.cleanup_confirmed
          assert not self.tx(TESTER_PRESENT)
          assert not self.tx(EXTENDED_SESSION)
          assert not self.tx(READ_DTCS)

  def test_cleanup_replay_drains_unacknowledged_request_before_two_fresh_acks(self):
    response = b"\x02\x50\x01\x00\x00\x00\x00\x00"
    for response_queued in (False, True):
      for reinitialize_panda in (False, True):
        with self.subTest(response_queued=response_queued, reinitialize_panda=reinitialize_panda):
          self.setUp()
          assert self.tx(DEFAULT_SESSION)
          if response_queued:
            self.rx(response)
          if reinitialize_panda:
            self.setUp()

          probe = RadarDiagnosticProbe()
          started_at = 0.1
          probe.start_cleanup(started_at)
          stale_packets = [CanData(RX_ADDR, response, BUS)] if response_queued else []
          deadline = started_at + probe.RECOVERY_DRAIN_TIMEOUT
          assert probe.update(stale_packets, deadline - 0.01).can_sends == ()

          assert self.advance_recovery(probe, deadline).state == RadarDiagnosticState.CLEANUP
          output = self.advance_recovery(probe, deadline + 0.1)
          assert output.state == RadarDiagnosticState.COMPLETE
          assert output.report is not None and output.report.cleanup_confirmed

  def test_restarting_during_cleanup_drain_restarts_the_quiet_period(self):
    self.setUp()
    assert self.tx(DEFAULT_SESSION)

    probe = RadarDiagnosticProbe()
    probe.start_cleanup(0.1)
    assert probe.update([], 2.0).can_sends == ()

    probe = RadarDiagnosticProbe()
    probe.start_cleanup(2.0)
    assert probe.update([], 5.49).can_sends == ()
    assert self.advance_recovery(probe, 5.5).state == RadarDiagnosticState.CLEANUP
    output = self.advance_recovery(probe, 5.6)
    assert output.state == RadarDiagnosticState.COMPLETE
    assert output.report is not None and output.report.cleanup_confirmed

  def test_only_exact_cleanup_is_replayable_after_completion(self):
    assert self.tx(DEFAULT_SESSION)
    self.single(b"\x50\x01")
    assert self.tx(DEFAULT_SESSION)
    self.single(b"\x50\x01")
    assert self.tx(DEFAULT_SESSION)
    self.single(b"\x50\x01")
    assert not self.tx(TESTER_PRESENT)
    assert not self.tx(b"\x02\x10\x01\x01\x00\x00\x00\x00")
    assert not self.tx(b"\x03\x10\x01\x00\x00\x00\x00\x00")
    assert not self.tx(EXTENDED_SESSION)
    assert not self.tx(READ_DTCS)

    self.setUp()
    self.safety.set_controls_allowed(True)
    assert not self.tx(DEFAULT_SESSION)


if __name__ == "__main__":
  unittest.main()
