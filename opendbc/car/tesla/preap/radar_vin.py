from dataclasses import dataclass
from enum import Enum, auto

from opendbc.car.can_definitions import CanData


RADAR_TX_ADDRESS = 0x641
RADAR_RX_ADDRESS = 0x651
RADAR_BUS = 1
RADAR_RX_OFFSET = RADAR_RX_ADDRESS - RADAR_TX_ADDRESS
VIN_DID_RESPONSE_PREFIX = b"\x62\xf1\x90"
ROUTINE_ID = b"\x0a\x03"
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"
LEGAL_VIN_CHARACTERS = frozenset("0123456789ABCDEFGHJKLMNPRSTUVWXYZ")


def _is_valid_vin(value: bytes) -> bool:
  try:
    vin = value.decode("ascii")
  except UnicodeDecodeError:
    return False
  return len(vin) == 17 and all(character in LEGAL_VIN_CHARACTERS for character in vin)


class RadarVinLearnerState(Enum):
  IDLE = auto()
  TESTER_PRESENT = auto()
  DEFAULT_SESSION = auto()
  EXTENDED_SESSION = auto()
  READINESS = auto()
  PRE_LEARN_VIN = auto()
  REQUEST_SEED = auto()
  SEND_KEY = auto()
  START_ROUTINE = auto()
  WAIT_STOP = auto()
  STOP_ROUTINE = auto()
  REQUEST_RESULTS = auto()
  POST_LEARN_VIN = auto()
  CLEANUP = auto()
  COMPLETE = auto()
  FAILED = auto()


class RadarVinLearnerResult(Enum):
  ALREADY_MATCHED = auto()
  LEARNED = auto()
  FAILED = auto()


class RadarVinFailure(Enum):
  ABORTED = auto()
  OVERALL_TIMEOUT = auto()
  TIMEOUT = auto()
  MALFORMED_RESPONSE = auto()
  UNEXPECTED_RESPONSE = auto()
  NEGATIVE_RESPONSE = auto()
  KEY_REJECTED = auto()
  POST_LEARN_MISMATCH = auto()


@dataclass(frozen=True)
class RadarVinLearnerOutput:
  state: RadarVinLearnerState
  can_sends: tuple[CanData, ...]
  result: RadarVinLearnerResult | None
  cleanup_confirmed: bool = False


@dataclass
class _ActiveRequest:
  payload: bytes
  positive_prefix: bytes
  sent_at: float
  timeout: float


class RadarVinAssembler:
  """Reassembles the three raw vehicle VIN mux fragments."""

  def __init__(self):
    self.fragments: dict[int, tuple[bytes, float]] = {}

  def update(self, can_packets: list[CanData], now: float) -> str | None:
    for packet in can_packets:
      if packet.address != 0x405 or packet.src != 0 or not packet.dat:
        continue

      mux = packet.dat[0]
      if mux not in (0x10, 0x11, 0x12):
        continue
      if len(packet.dat) != 8:
        self.fragments.pop(mux, None)
        continue

      expected_length = 3 if mux == 0x10 else 7
      fragment = packet.dat[5:8] if mux == 0x10 else packet.dat[1:8]
      if len(fragment) != expected_length:
        self.fragments.pop(mux, None)
        continue
      self.fragments[mux] = (fragment, now)

    if not all(mux in self.fragments for mux in (0x10, 0x11, 0x12)):
      return None

    ordered_fragments = [self.fragments[mux] for mux in (0x10, 0x11, 0x12)]
    if any(not 0.0 <= now - received_at <= 1.0 for _, received_at in ordered_fragments):
      return None

    raw_vin = b"".join(fragment for fragment, _ in ordered_fragments)
    if not _is_valid_vin(raw_vin):
      return None
    return raw_vin.decode("ascii")


def tesla_radar_security_key(seed: bytes) -> bytes:
  """Return the four-byte Tesla radar SecurityAccess key for a four-byte seed."""
  if len(seed) != 4:
    raise ValueError("Tesla radar security seed must be four bytes")

  value = int.from_bytes(seed, "big")
  mask = 0xFFFFFFFF
  rotation = ((value >> 5) & 8) | ((value >> 11) & 4) | ((value >> 24) & 1) | ((value >> 1) & 2)

  if (value & 0x20000) == 0:
    transformed = (((value & ~((0xFF << rotation) & mask)) << (32 - rotation)) & mask) | ((value >> rotation) & mask)
  else:
    transformed = (((((~((0xFF << rotation) & mask)) << (32 - rotation)) & value & mask) >> (32 - rotation)) & mask) | ((value << rotation) & mask)

  selector = ((value >> 4) & 2) | (value >> 31)
  if selector == 0:
    key = transformed | value
  elif selector == 1:
    key = transformed & value
  elif selector == 2:
    key = transformed ^ value
  else:
    key = transformed
  return key.to_bytes(4, "big")


class RadarVinLearner:
  RESPONSE_TIMEOUT = 3.0
  CLOCK_EPSILON = 1e-9
  READINESS_INTERVAL = 0.1
  READINESS_ATTEMPTS = 10
  STOP_INTERVAL = 2.0
  STOP_ATTEMPTS = 3
  OVERALL_TIMEOUT = 30.0

  def __init__(self):
    self.state = RadarVinLearnerState.IDLE
    self.failure: RadarVinFailure | None = None
    self.result: RadarVinLearnerResult | None = None
    self._target_vin = b""
    self._started_at = 0.0
    self._next_action_at = 0.0
    self._active_request: _ActiveRequest | None = None
    self._key: bytes | None = None
    self._readiness_attempts = 0
    self._stop_attempts = 0
    self._successful_stops = 0
    self._ecu_responsive = False
    self._cleanup_confirmed = False
    self._pending_result: RadarVinLearnerResult | None = None
    self._rx_payload = bytearray()
    self._rx_length: int | None = None
    self._rx_sequence = 1

  def start(self, target_vin: str, now: float) -> None:
    try:
      encoded_target = target_vin.encode("ascii")
    except UnicodeEncodeError as exc:
      raise ValueError("target VIN is invalid") from exc
    if not _is_valid_vin(encoded_target):
      raise ValueError("target VIN is invalid")

    self.state = RadarVinLearnerState.TESTER_PRESENT
    self.failure = None
    self.result = None
    self._target_vin = encoded_target
    self._started_at = now
    self._next_action_at = now
    self._active_request = None
    self._key = None
    self._readiness_attempts = 0
    self._stop_attempts = 0
    self._successful_stops = 0
    self._ecu_responsive = False
    self._cleanup_confirmed = False
    self._pending_result = None
    self._reset_rx()

  def update(self, can_packets: list[CanData], now: float) -> RadarVinLearnerOutput:
    """Process newly received CAN frames in chronological order for this update only.

    Callers must not replay frames from earlier updates. The learner does not
    retain completed responses or correlate responses beyond the active request.
    """
    can_sends: list[CanData] = []
    if self.state in (RadarVinLearnerState.IDLE, RadarVinLearnerState.COMPLETE, RadarVinLearnerState.FAILED):
      return self._output(can_sends)

    if self.state != RadarVinLearnerState.CLEANUP and now - self._started_at + self.CLOCK_EPSILON >= self.OVERALL_TIMEOUT:
      self._fail(RadarVinFailure.OVERALL_TIMEOUT)
      return self._output(can_sends)

    if self._active_request is not None:
      if now - self._active_request.sent_at + self.CLOCK_EPSILON >= self._active_request.timeout:
        if self.state == RadarVinLearnerState.READINESS and self._readiness_attempts < self.READINESS_ATTEMPTS:
          self._active_request = None
          self._reset_rx()
        else:
          self._fail(RadarVinFailure.TIMEOUT)
          return self._output(can_sends)
      else:
        for packet in can_packets:
          if packet.address != RADAR_RX_ADDRESS or packet.src != RADAR_BUS:
            continue
          payload, flow_control, malformed = self._consume_isotp(packet.dat)
          if flow_control:
            can_sends.append(self._make_send(FLOW_CONTROL))
          if malformed:
            self._fail(RadarVinFailure.MALFORMED_RESPONSE)
            return self._output(can_sends)
          if payload is not None:
            self._handle_payload(payload, now)
            return self._output(can_sends)

        return self._output(can_sends)

    if self.state == RadarVinLearnerState.WAIT_STOP:
      if now < self._next_action_at:
        return self._output(can_sends)
      self.state = RadarVinLearnerState.STOP_ROUTINE

    if now < self._next_action_at:
      return self._output(can_sends)

    request = self._request_for_state()
    if request is not None:
      payload, prefix, timeout = request
      can_sends.append(self._send_request(payload, prefix, timeout, now))
    return self._output(can_sends)

  def abort(self, reason: RadarVinFailure, now: float) -> RadarVinLearnerOutput:
    if self.state not in (RadarVinLearnerState.IDLE, RadarVinLearnerState.COMPLETE, RadarVinLearnerState.FAILED):
      self._fail(reason)
    return self._output([])

  def _request_for_state(self) -> tuple[bytes, bytes, float] | None:
    if self.state in (RadarVinLearnerState.TESTER_PRESENT, RadarVinLearnerState.READINESS):
      timeout = self.READINESS_INTERVAL if self.state == RadarVinLearnerState.READINESS else self.RESPONSE_TIMEOUT
      return b"\x3e\x00", b"\x7e\x00", timeout
    if self.state in (RadarVinLearnerState.DEFAULT_SESSION, RadarVinLearnerState.CLEANUP):
      return b"\x10\x01", b"\x50\x01", self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.EXTENDED_SESSION:
      return b"\x10\x03", b"\x50\x03", self.RESPONSE_TIMEOUT
    if self.state in (RadarVinLearnerState.PRE_LEARN_VIN, RadarVinLearnerState.POST_LEARN_VIN):
      return b"\x22\xf1\x90", VIN_DID_RESPONSE_PREFIX, self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.REQUEST_SEED:
      return b"\x27\x11", b"\x67\x11", self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.SEND_KEY:
      if self._key is None:
        self._fail(RadarVinFailure.MALFORMED_RESPONSE)
        return None
      return b"\x27\x12" + self._key, b"\x67\x12", self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.START_ROUTINE:
      return b"\x31\x01" + ROUTINE_ID, b"\x71\x01" + ROUTINE_ID, self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.STOP_ROUTINE:
      return b"\x31\x02" + ROUTINE_ID, b"\x71\x02" + ROUTINE_ID, self.RESPONSE_TIMEOUT
    if self.state == RadarVinLearnerState.REQUEST_RESULTS:
      return b"\x31\x03" + ROUTINE_ID, b"\x71\x03" + ROUTINE_ID, self.RESPONSE_TIMEOUT
    return None

  def _send_request(self, payload: bytes, prefix: bytes, timeout: float, now: float) -> CanData:
    if len(payload) > 7:
      raise ValueError("multi-frame host requests are prohibited")
    if self.state == RadarVinLearnerState.READINESS:
      self._readiness_attempts += 1
    if self.state == RadarVinLearnerState.STOP_ROUTINE:
      self._stop_attempts += 1
    self._active_request = _ActiveRequest(payload, prefix, now, timeout)
    return self._make_send(bytes([len(payload)]) + payload)

  def _make_send(self, data: bytes) -> CanData:
    return CanData(RADAR_TX_ADDRESS, data.ljust(8, b"\x00"), RADAR_BUS)

  def _consume_isotp(self, data: bytes) -> tuple[bytes | None, bool, bool]:
    if len(data) != 8:
      return None, False, True

    frame_type = data[0] >> 4
    if frame_type == 0:
      length = data[0] & 0x0F
      if length > 7:
        return None, False, True
      return data[1:1 + length], False, False

    if frame_type == 1:
      total_length = ((data[0] & 0x0F) << 8) | data[1]
      if total_length <= 7:
        return None, False, True
      self._rx_payload = bytearray(data[2:])
      self._rx_length = total_length
      self._rx_sequence = 1
      if len(self._rx_payload) >= total_length:
        self._reset_rx()
        return None, False, True
      if self._active_request is None or not self._rx_prefix_is_valid(bytes(self._rx_payload)):
        self._reset_rx()
        return None, False, True
      return None, True, False

    if frame_type == 2:
      if self._rx_length is None or (data[0] & 0x0F) != self._rx_sequence:
        self._reset_rx()
        return None, False, True
      self._rx_sequence = (self._rx_sequence + 1) & 0x0F
      remaining_length = self._rx_length - len(self._rx_payload)
      self._rx_payload.extend(data[1:1 + remaining_length])
      if len(self._rx_payload) < self._rx_length:
        return None, False, False
      payload = bytes(self._rx_payload)
      self._reset_rx()
      return payload, False, False

    return None, False, True

  def _rx_prefix_is_valid(self, payload: bytes) -> bool:
    assert self._active_request is not None
    prefix = self._active_request.positive_prefix
    return len(payload) >= len(prefix) and payload.startswith(prefix)

  def _handle_payload(self, payload: bytes, now: float) -> None:
    assert self._active_request is not None
    request = self._active_request
    if payload.startswith(b"\x7f"):
      self._handle_negative_response(payload, request.payload[0], now)
      return
    if not payload.startswith(request.positive_prefix):
      self._fail(RadarVinFailure.UNEXPECTED_RESPONSE)
      return
    if not self._positive_response_is_valid(payload):
      self._fail(RadarVinFailure.MALFORMED_RESPONSE)
      return

    self._ecu_responsive = True
    self._active_request = None
    if self.state == RadarVinLearnerState.TESTER_PRESENT:
      self.state = RadarVinLearnerState.DEFAULT_SESSION
    elif self.state == RadarVinLearnerState.DEFAULT_SESSION:
      self.state = RadarVinLearnerState.EXTENDED_SESSION
    elif self.state == RadarVinLearnerState.EXTENDED_SESSION:
      self.state = RadarVinLearnerState.READINESS
    elif self.state == RadarVinLearnerState.READINESS:
      self.state = RadarVinLearnerState.PRE_LEARN_VIN
    elif self.state == RadarVinLearnerState.PRE_LEARN_VIN:
      self._handle_pre_learn_vin(payload[3:])
    elif self.state == RadarVinLearnerState.REQUEST_SEED:
      self._handle_seed(payload[2:])
    elif self.state == RadarVinLearnerState.SEND_KEY:
      self.state = RadarVinLearnerState.START_ROUTINE
    elif self.state == RadarVinLearnerState.START_ROUTINE:
      self.state = RadarVinLearnerState.WAIT_STOP
      self._next_action_at = now + self.STOP_INTERVAL
    elif self.state == RadarVinLearnerState.STOP_ROUTINE:
      self._successful_stops += 1
      if self._successful_stops == 2:
        self.state = RadarVinLearnerState.REQUEST_RESULTS
      else:
        self._stop_attempts = 0
        self.state = RadarVinLearnerState.WAIT_STOP
        self._next_action_at = now + self.STOP_INTERVAL
    elif self.state == RadarVinLearnerState.REQUEST_RESULTS:
      self.state = RadarVinLearnerState.POST_LEARN_VIN
    elif self.state == RadarVinLearnerState.POST_LEARN_VIN:
      learned_vin = payload[3:]
      if not _is_valid_vin(learned_vin):
        self._fail(RadarVinFailure.MALFORMED_RESPONSE)
      elif learned_vin == self._target_vin:
        self._pending_result = RadarVinLearnerResult.LEARNED
        self._enter_cleanup()
      else:
        self._fail(RadarVinFailure.POST_LEARN_MISMATCH)
    elif self.state == RadarVinLearnerState.CLEANUP:
      self._cleanup_confirmed = True
      if self._pending_result in (RadarVinLearnerResult.ALREADY_MATCHED, RadarVinLearnerResult.LEARNED):
        self.state = RadarVinLearnerState.COMPLETE
        self.result = self._pending_result
      else:
        self._finish_failure()

  def _positive_response_is_valid(self, payload: bytes) -> bool:
    if self.state in (RadarVinLearnerState.DEFAULT_SESSION, RadarVinLearnerState.EXTENDED_SESSION, RadarVinLearnerState.CLEANUP):
      return len(payload) >= 2
    if self.state in (RadarVinLearnerState.TESTER_PRESENT, RadarVinLearnerState.READINESS, RadarVinLearnerState.SEND_KEY):
      return len(payload) == 2
    if self.state in (RadarVinLearnerState.PRE_LEARN_VIN, RadarVinLearnerState.POST_LEARN_VIN):
      return len(payload) == 20
    if self.state == RadarVinLearnerState.REQUEST_SEED:
      return len(payload) == 6
    if self.state in (RadarVinLearnerState.START_ROUTINE, RadarVinLearnerState.STOP_ROUTINE, RadarVinLearnerState.REQUEST_RESULTS):
      return len(payload) >= 4
    return False

  def _handle_pre_learn_vin(self, vin: bytes) -> None:
    if not _is_valid_vin(vin):
      self._fail(RadarVinFailure.MALFORMED_RESPONSE)
      return
    if vin == self._target_vin:
      self._pending_result = RadarVinLearnerResult.ALREADY_MATCHED
      self._enter_cleanup()
    else:
      self.state = RadarVinLearnerState.REQUEST_SEED

  def _handle_seed(self, seed: bytes) -> None:
    try:
      self._key = tesla_radar_security_key(seed)
    except ValueError:
      self._fail(RadarVinFailure.MALFORMED_RESPONSE)
      return
    self.state = RadarVinLearnerState.SEND_KEY

  def _handle_negative_response(self, payload: bytes, request_service: int, now: float) -> None:
    if len(payload) != 3 or payload[1] != request_service:
      self._fail(RadarVinFailure.MALFORMED_RESPONSE)
      return
    self._ecu_responsive = True
    if payload[2] == 0x78:
      return
    if self.state == RadarVinLearnerState.STOP_ROUTINE and self._stop_attempts < self.STOP_ATTEMPTS:
      self._active_request = None
      self.state = RadarVinLearnerState.WAIT_STOP
      self._next_action_at = now + self.STOP_INTERVAL
      return
    if self.state == RadarVinLearnerState.SEND_KEY:
      self._fail(RadarVinFailure.KEY_REJECTED)
      return
    self._fail(RadarVinFailure.NEGATIVE_RESPONSE)

  def _fail(self, reason: RadarVinFailure) -> None:
    self.failure = reason
    self._pending_result = RadarVinLearnerResult.FAILED
    self._active_request = None
    self._reset_rx()
    self._enter_cleanup()

  def _enter_cleanup(self) -> None:
    if self.state == RadarVinLearnerState.CLEANUP:
      self._finish_failure()
    elif self._ecu_responsive:
      self.state = RadarVinLearnerState.CLEANUP
      self._next_action_at = self._started_at
    else:
      self._finish_failure()

  def _finish_failure(self) -> None:
    self.state = RadarVinLearnerState.FAILED
    self.result = RadarVinLearnerResult.FAILED
    self._active_request = None
    self._reset_rx()

  def _reset_rx(self) -> None:
    self._rx_payload.clear()
    self._rx_length = None
    self._rx_sequence = 1

  def _output(self, can_sends: list[CanData]) -> RadarVinLearnerOutput:
    result = self.result if self.state in (RadarVinLearnerState.COMPLETE, RadarVinLearnerState.FAILED) else None
    return RadarVinLearnerOutput(self.state, tuple(can_sends), result, self._cleanup_confirmed)
