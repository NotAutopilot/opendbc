from dataclasses import dataclass
from enum import Enum, auto

from opendbc.car.can_definitions import CanData


RADAR_DIAGNOSTIC_TX_ADDRESS = 0x641
RADAR_DIAGNOSTIC_RX_ADDRESS = 0x651
RADAR_DIAGNOSTIC_BUS = 1
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"

RADAR_IDENTITY_DIDS = (
  0xA022,
  0xF014,
  0xF015,
  0xF180,
  0xF181,
  0xF182,
  0xF187,
  0xF188,
  0xF189,
  0xF18A,
  0xF18C,
  0xF191,
  0xF192,
  0xF193,
  0xF194,
  0xF195,
  0xF197,
  0xF19E,
)


class RadarDiagnosticState(Enum):
  IDLE = auto()
  TESTER_PRESENT = auto()
  DEFAULT_SESSION = auto()
  EXTENDED_SESSION = auto()
  READINESS = auto()
  READ_DID = auto()
  READ_DTCS = auto()
  READ_DTC_SNAPSHOT = auto()
  READ_DTC_EXTENDED_DATA = auto()
  CLEANUP = auto()
  COMPLETE = auto()
  FAILED = auto()


class RadarDiagnosticFailure(Enum):
  ABORTED = auto()
  OVERALL_TIMEOUT = auto()
  TIMEOUT = auto()
  MALFORMED_RESPONSE = auto()
  UNEXPECTED_RESPONSE = auto()
  NEGATIVE_RESPONSE = auto()


class RadarIsoTpResponseState(Enum):
  INCOMPLETE = auto()
  COMPLETE = auto()
  MALFORMED = auto()


@dataclass(frozen=True)
class RadarIsoTpResponse:
  state: RadarIsoTpResponseState
  payload: bytes | None = None
  can_sends: tuple[CanData, ...] = ()
  prefix_mismatch: bool = False
  multiframe: bool = False


class RadarIsoTpResponseAssembler:
  def __init__(self, max_payload_length: int = 0xFFF):
    if not 8 <= max_payload_length <= 0xFFF:
      raise ValueError("ISO-TP response limit must be between 8 and 4095 bytes")
    self.max_payload_length = max_payload_length
    self.reset()

  def consume(self, data: bytes, expected_prefixes: tuple[bytes, ...]) -> RadarIsoTpResponse:
    if len(data) != 8:
      return self._malformed()
    frame_type = data[0] >> 4
    if frame_type == 0:
      length = data[0] & 0x0F
      if self._length is not None or not 1 <= length <= 7:
        return self._malformed()
      payload = data[1 : 1 + length]
      if not self._prefix_matches(payload, expected_prefixes):
        return self._malformed(payload=payload, prefix_mismatch=True)
      return RadarIsoTpResponse(RadarIsoTpResponseState.COMPLETE, payload)
    if frame_type == 1:
      length = ((data[0] & 0x0F) << 8) | data[1]
      if self._length is not None or not 7 < length <= self.max_payload_length:
        return self._malformed(multiframe=True)
      payload_prefix = data[2:]
      if not self._prefix_matches(payload_prefix, expected_prefixes):
        return self._malformed(payload=payload_prefix, prefix_mismatch=True, multiframe=True)
      self._payload = bytearray(payload_prefix)
      self._length = length
      self._sequence = 1
      return RadarIsoTpResponse(
        RadarIsoTpResponseState.INCOMPLETE, can_sends=(CanData(RADAR_DIAGNOSTIC_TX_ADDRESS, FLOW_CONTROL, RADAR_DIAGNOSTIC_BUS),), multiframe=True
      )
    if frame_type == 2:
      sequence = data[0] & 0x0F
      if self._length is None or sequence != self._sequence:
        return self._malformed(multiframe=True)
      self._sequence = (self._sequence + 1) & 0x0F
      remaining = self._length - len(self._payload)
      self._payload.extend(data[1 : 1 + remaining])
      if len(self._payload) < self._length:
        return RadarIsoTpResponse(RadarIsoTpResponseState.INCOMPLETE, multiframe=True)
      payload = bytes(self._payload)
      self.reset()
      return RadarIsoTpResponse(RadarIsoTpResponseState.COMPLETE, payload, multiframe=True)
    return self._malformed(multiframe=self._length is not None)

  def reset(self) -> None:
    self._payload = bytearray()
    self._length: int | None = None
    self._sequence = 1

  @staticmethod
  def _prefix_matches(payload: bytes, expected_prefixes: tuple[bytes, ...]) -> bool:
    return not expected_prefixes or any(payload.startswith(prefix) for prefix in expected_prefixes)

  def _malformed(self, *, payload: bytes | None = None, prefix_mismatch: bool = False, multiframe: bool = False) -> RadarIsoTpResponse:
    self.reset()
    return RadarIsoTpResponse(RadarIsoTpResponseState.MALFORMED, payload, prefix_mismatch=prefix_mismatch, multiframe=multiframe)


@dataclass(frozen=True)
class RadarIdentityRecord:
  identifier: int
  value: bytes | None = None
  negative_response_code: int | None = None


@dataclass(frozen=True)
class RadarDtcDetailRecord:
  code: bytes
  snapshot: bytes | None = None
  snapshot_negative_response_code: int | None = None
  extended_data: bytes | None = None
  extended_data_negative_response_code: int | None = None


@dataclass(frozen=True)
class RadarDiagnosticReport:
  records: tuple[RadarIdentityRecord, ...]
  dtc_response: bytes | None
  dtc_negative_response_code: int | None
  dtc_details: tuple[RadarDtcDetailRecord, ...]
  failure: RadarDiagnosticFailure | None
  cleanup_confirmed: bool
  dtc_status_availability_mask: int | None = None


@dataclass(frozen=True)
class RadarDiagnosticOutput:
  state: RadarDiagnosticState
  can_sends: tuple[CanData, ...]
  report: RadarDiagnosticReport | None


@dataclass
class _ActiveRequest:
  payload: bytes
  positive_prefix: bytes
  sent_at: float
  timeout: float


class RadarDiagnosticProbe:
  """Collect a bounded, read-only radar identity and DTC report."""

  RESPONSE_TIMEOUT = 3.0
  RESPONSE_PENDING_TIMEOUT = 5.0
  CLEANUP_TIMEOUT = 3.0
  OVERALL_TIMEOUT = 45.0
  MAX_DTC_DETAILS = 16
  MAX_DTC_DETAIL_RESPONSE_LENGTH = 256
  CLOCK_EPSILON = 1e-9

  def __init__(self, *, include_dtc_details: bool = True):
    self.identifiers = RADAR_IDENTITY_DIDS
    self.include_dtc_details = include_dtc_details
    self.state = RadarDiagnosticState.IDLE
    self.failure: RadarDiagnosticFailure | None = None
    self._response_assembler = RadarIsoTpResponseAssembler()

  def start(self, now: float) -> None:
    self.state = RadarDiagnosticState.TESTER_PRESENT
    self.failure = None
    self._started_at = now
    self._cleanup_deadline_at: float | None = None
    self._active_request: _ActiveRequest | None = None
    self._identifier_index = 0
    self._records: list[RadarIdentityRecord] = []
    self._dtc_response: bytes | None = None
    self._dtc_nrc: int | None = None
    self._dtc_mask: int | None = None
    self._dtc_codes: list[bytes] = []
    self._dtc_details: list[RadarDtcDetailRecord] = []
    self._detail_index = 0
    self._ecu_responsive = False
    self._cleanup_confirmed = False
    self._cleanup_acks_remaining = 1
    self._response_assembler.reset()

  def start_cleanup(self, now: float) -> None:
    """Recover a persisted transaction by issuing only the default-session cleanup."""
    self.start(now)
    self._cleanup_acks_remaining = 2
    self.state = RadarDiagnosticState.CLEANUP

  def abort(self, now: float) -> RadarDiagnosticOutput:
    if self.state not in (RadarDiagnosticState.IDLE, RadarDiagnosticState.COMPLETE, RadarDiagnosticState.FAILED):
      self._fail(RadarDiagnosticFailure.ABORTED, now)
    return self._output(())

  def update(self, can_packets: list[CanData], now: float) -> RadarDiagnosticOutput:
    can_sends: list[CanData] = []
    if self.state in (RadarDiagnosticState.IDLE, RadarDiagnosticState.COMPLETE, RadarDiagnosticState.FAILED):
      return self._output(can_sends)
    if self.state != RadarDiagnosticState.CLEANUP and now - self._started_at + self.CLOCK_EPSILON >= self.OVERALL_TIMEOUT:
      self._fail(RadarDiagnosticFailure.OVERALL_TIMEOUT, now)
      return self._output(can_sends)
    if self.state == RadarDiagnosticState.CLEANUP and self._cleanup_deadline_at is not None and now >= self._cleanup_deadline_at:
      self._fail(RadarDiagnosticFailure.TIMEOUT, now)
      return self._output(can_sends)
    if self._active_request is not None:
      if now - self._active_request.sent_at + self.CLOCK_EPSILON >= self._active_request.timeout:
        self._fail(RadarDiagnosticFailure.TIMEOUT, now)
        return self._output(can_sends)
      for packet in can_packets:
        if packet.address != RADAR_DIAGNOSTIC_RX_ADDRESS or packet.src != RADAR_DIAGNOSTIC_BUS:
          continue
        expected = (self._active_request.positive_prefix,)
        if len(packet.dat) == 8 and packet.dat[0] >> 4 == 0:
          expected += (b"\x7f",)
        response = self._response_assembler.consume(packet.dat, expected)
        can_sends.extend(response.can_sends)
        if response.state == RadarIsoTpResponseState.MALFORMED:
          self._fail(
            RadarDiagnosticFailure.UNEXPECTED_RESPONSE if response.prefix_mismatch and not response.multiframe else RadarDiagnosticFailure.MALFORMED_RESPONSE,
            now,
          )
          return self._output(can_sends)
        if response.state == RadarIsoTpResponseState.COMPLETE:
          self._handle_payload(response.payload or b"", now)
          return self._output(can_sends)
      return self._output(can_sends)
    request = self._request_for_state()
    if request is not None:
      payload, prefix = request
      max_length = (
        self.MAX_DTC_DETAIL_RESPONSE_LENGTH if self.state in (RadarDiagnosticState.READ_DTC_SNAPSHOT, RadarDiagnosticState.READ_DTC_EXTENDED_DATA) else 0xFFF
      )
      self._response_assembler = RadarIsoTpResponseAssembler(max_length)
      self._active_request = _ActiveRequest(payload, prefix, now, self.RESPONSE_TIMEOUT)
      if self.state == RadarDiagnosticState.CLEANUP:
        self._cleanup_deadline_at = now + self.CLEANUP_TIMEOUT
      can_sends.append(CanData(RADAR_DIAGNOSTIC_TX_ADDRESS, (bytes([len(payload)]) + payload).ljust(8, b"\x00"), RADAR_DIAGNOSTIC_BUS))
    return self._output(can_sends)

  def _request_for_state(self) -> tuple[bytes, bytes] | None:
    if self.state in (RadarDiagnosticState.TESTER_PRESENT, RadarDiagnosticState.READINESS):
      return b"\x3e\x00", b"\x7e\x00"
    if self.state in (RadarDiagnosticState.DEFAULT_SESSION, RadarDiagnosticState.CLEANUP):
      return b"\x10\x01", b"\x50\x01"
    if self.state == RadarDiagnosticState.EXTENDED_SESSION:
      return b"\x10\x03", b"\x50\x03"
    if self.state == RadarDiagnosticState.READ_DID:
      did = self.identifiers[self._identifier_index].to_bytes(2, "big")
      return b"\x22" + did, b"\x62" + did
    if self.state == RadarDiagnosticState.READ_DTCS:
      return b"\x19\x02\xff", b"\x59\x02"
    if self.state == RadarDiagnosticState.READ_DTC_SNAPSHOT:
      return b"\x19\x04" + self._dtc_codes[self._detail_index] + b"\xff", b"\x59\x04" + self._dtc_codes[self._detail_index]
    if self.state == RadarDiagnosticState.READ_DTC_EXTENDED_DATA:
      return b"\x19\x06" + self._dtc_codes[self._detail_index] + b"\xff", b"\x59\x06" + self._dtc_codes[self._detail_index]
    return None

  def _handle_payload(self, payload: bytes, now: float) -> None:
    assert self._active_request is not None
    request = self._active_request
    if payload.startswith(b"\x7f"):
      self._handle_negative_response(payload, request.payload[0], now)
      return
    if not payload.startswith(request.positive_prefix):
      self._fail(RadarDiagnosticFailure.UNEXPECTED_RESPONSE, now)
      return
    self._ecu_responsive = True
    self._active_request = None
    if self.state in (RadarDiagnosticState.TESTER_PRESENT, RadarDiagnosticState.READINESS):
      if len(payload) != 2:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        self.state = (
          RadarDiagnosticState.DEFAULT_SESSION
          if self.state == RadarDiagnosticState.TESTER_PRESENT
          else (RadarDiagnosticState.READ_DID if self.identifiers else RadarDiagnosticState.READ_DTCS)
        )
    elif self.state == RadarDiagnosticState.DEFAULT_SESSION:
      if len(payload) < 2:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        self.state = RadarDiagnosticState.EXTENDED_SESSION
    elif self.state == RadarDiagnosticState.EXTENDED_SESSION:
      if len(payload) < 2:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        self.state = RadarDiagnosticState.READINESS
    elif self.state == RadarDiagnosticState.READ_DID:
      self._records.append(RadarIdentityRecord(self.identifiers[self._identifier_index], payload[3:]))
      self._identifier_index += 1
      self.state = RadarDiagnosticState.READ_DID if self._identifier_index < len(self.identifiers) else RadarDiagnosticState.READ_DTCS
    elif self.state == RadarDiagnosticState.READ_DTCS:
      if len(payload) < 3 or (len(payload) - 3) % 4:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        self._dtc_response, self._dtc_mask = payload, payload[2]
        self._dtc_codes = [payload[index : index + 3] for index in range(3, len(payload), 4)][: self.MAX_DTC_DETAILS]
        self._dtc_details = [RadarDtcDetailRecord(code) for code in self._dtc_codes] if self.include_dtc_details else []
        self.state = RadarDiagnosticState.READ_DTC_SNAPSHOT if self._dtc_details else RadarDiagnosticState.CLEANUP
    elif self.state == RadarDiagnosticState.READ_DTC_SNAPSHOT:
      if len(payload) < 6:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        detail = self._dtc_details[self._detail_index]
        self._dtc_details[self._detail_index] = RadarDtcDetailRecord(detail.code, snapshot=payload[5:])
        self.state = RadarDiagnosticState.READ_DTC_EXTENDED_DATA
    elif self.state == RadarDiagnosticState.READ_DTC_EXTENDED_DATA:
      if len(payload) < 6:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        detail = self._dtc_details[self._detail_index]
        self._dtc_details[self._detail_index] = RadarDtcDetailRecord(detail.code, detail.snapshot, detail.snapshot_negative_response_code, payload[5:])
        self._advance_detail()
    elif self.state == RadarDiagnosticState.CLEANUP:
      if len(payload) < 2:
        self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      else:
        self._cleanup_acks_remaining -= 1
        if self._cleanup_acks_remaining > 0:
          self.state = RadarDiagnosticState.CLEANUP
        else:
          self._cleanup_confirmed = True
          self.state = RadarDiagnosticState.FAILED if self.failure else RadarDiagnosticState.COMPLETE

  def _handle_negative_response(self, payload: bytes, request_sid: int, now: float) -> None:
    if len(payload) != 3 or payload[1] != request_sid:
      self._fail(RadarDiagnosticFailure.MALFORMED_RESPONSE, now)
      return
    self._ecu_responsive = True
    if payload[2] == 0x78:
      assert self._active_request is not None
      self._active_request.sent_at, self._active_request.timeout = now, self.RESPONSE_PENDING_TIMEOUT
      return
    self._active_request = None
    if self.state == RadarDiagnosticState.READ_DID:
      self._records.append(RadarIdentityRecord(self.identifiers[self._identifier_index], negative_response_code=payload[2]))
      self._identifier_index += 1
      self.state = RadarDiagnosticState.READ_DID if self._identifier_index < len(self.identifiers) else RadarDiagnosticState.READ_DTCS
    elif self.state == RadarDiagnosticState.READ_DTCS:
      self._dtc_nrc, self.state = payload[2], RadarDiagnosticState.CLEANUP
    elif self.state == RadarDiagnosticState.READ_DTC_SNAPSHOT:
      detail = self._dtc_details[self._detail_index]
      self._dtc_details[self._detail_index] = RadarDtcDetailRecord(detail.code, snapshot_negative_response_code=payload[2])
      self.state = RadarDiagnosticState.READ_DTC_EXTENDED_DATA
    elif self.state == RadarDiagnosticState.READ_DTC_EXTENDED_DATA:
      detail = self._dtc_details[self._detail_index]
      self._dtc_details[self._detail_index] = RadarDtcDetailRecord(
        detail.code, detail.snapshot, detail.snapshot_negative_response_code, extended_data_negative_response_code=payload[2]
      )
      self._advance_detail()
    else:
      self._fail(RadarDiagnosticFailure.NEGATIVE_RESPONSE, now)

  def _advance_detail(self) -> None:
    self._detail_index += 1
    self.state = RadarDiagnosticState.READ_DTC_SNAPSHOT if self._detail_index < len(self._dtc_details) else RadarDiagnosticState.CLEANUP

  def _fail(self, failure: RadarDiagnosticFailure, now: float) -> None:
    if self.failure is None:
      self.failure = failure
    self._active_request = None
    self._response_assembler.reset()
    self.state = RadarDiagnosticState.FAILED if self.state == RadarDiagnosticState.CLEANUP else RadarDiagnosticState.CLEANUP

  def _output(self, can_sends: list[CanData] | tuple[CanData, ...]) -> RadarDiagnosticOutput:
    report = None
    if self.state in (RadarDiagnosticState.COMPLETE, RadarDiagnosticState.FAILED):
      report = RadarDiagnosticReport(
        tuple(self._records), self._dtc_response, self._dtc_nrc, tuple(self._dtc_details), self.failure, self._cleanup_confirmed, self._dtc_mask
      )
    return RadarDiagnosticOutput(self.state, tuple(can_sends), report)
