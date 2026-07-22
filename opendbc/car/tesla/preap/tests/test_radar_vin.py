import logging

import pytest

from opendbc.car.can_definitions import CanData


TARGET_VIN = "ABCDEFGHJKLMNPRST"
OTHER_VIN = "ABCDEFGHJKLMNPRSU"
RADAR_TX_ADDRESS = 0x641
RADAR_RX_ADDRESS = 0x651
RADAR_BUS = 1


def _can(address, dat, bus=RADAR_BUS):
  return CanData(address, bytes(dat).ljust(8, b"\x00"), bus)


def _rx_single(payload):
  return _can(RADAR_RX_ADDRESS, bytes([len(payload)]) + payload)


def _rx_first(payload):
  return _can(RADAR_RX_ADDRESS, bytes([0x10 | (len(payload) >> 8), len(payload) & 0xFF]) + payload[:6])


def _rx_consecutive(sequence, payload):
  return _can(RADAR_RX_ADDRESS, bytes([0x20 | sequence]) + payload)


def _tx_payload(output):
  assert len(output.can_sends) == 1
  frame = output.can_sends[0]
  assert frame.address == RADAR_TX_ADDRESS
  assert frame.src == RADAR_BUS
  return frame.dat


def _respond(learner, payload, now):
  if len(payload) <= 7:
    frames = [_rx_single(payload)]
    expected_sends = ()
  else:
    frames = [_rx_first(payload)]
    frames.extend(_rx_consecutive(sequence, payload[start:start + 7])
                  for sequence, start in enumerate(range(6, len(payload), 7), start=1))
    expected_sends = (_can(RADAR_TX_ADDRESS, b"\x30\x00\x00\x00\x00\x00\x00\x00"),)
  response = learner.update(frames, now)
  assert response.can_sends == expected_sends
  return now + 0.01


def _prepare_pre_learn_vin(learner):
  now = 0.0
  learner.start(TARGET_VIN, now)
  assert _tx_payload(learner.update([], now)) == b"\x02\x3e\x00\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x7e\x00", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x50\x01\x00\x32\x01\xf4", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x02\x10\x03\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x50\x03\x00\x32\x01\xf4", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x02\x3e\x00\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x7e\x00", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x03\x22\xf1\x90\x00\x00\x00\x00"
  return now


def _prepare_key_request(learner):
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)
  learner.update([], now)
  return now


def _prepare_start_routine(learner):
  now = _prepare_key_request(learner)
  now = _respond(learner, b"\x67\x12", now + 0.01)
  learner.update([], now)
  return now


def _prepare_post_learn_vin(learner):
  now = _prepare_start_routine(learner)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)
  now += 2.0
  learner.update([], now)
  now = _respond(learner, b"\x71\x02\x0a\x03", now + 0.01)
  now += 2.0
  learner.update([], now)
  now = _respond(learner, b"\x71\x02\x0a\x03", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x71\x03\x0a\x03", now + 0.01)
  learner.update([], now)
  return now


def _module():
  try:
    from opendbc.car.tesla.preap import radar_vin
  except ModuleNotFoundError as exc:
    pytest.fail(f"radar_vin module is missing: {exc}")
  return radar_vin


def test_tesla_radar_security_key_vectors_and_length():
  radar_vin = _module()

  expected_vectors = {
    "00000000": "00000000",
    "12345678": "00000000",
    "0009a5e1": "5e19a57b",
    "deadbeef": "bf7ab6fb",
    "11223344": "19a23bd5",
    "89abcdef": "5e6f7c4d",
    "ffffffff": "ffffffff",
  }

  for seed_hex, key_hex in expected_vectors.items():
    assert radar_vin.tesla_radar_security_key(bytes.fromhex(seed_hex)) == bytes.fromhex(key_hex)

  with pytest.raises(ValueError):
    radar_vin.tesla_radar_security_key(b"")

  for invalid_seed in (b"\x00", b"\x00\x01\x02", b"\x00" * 5):
    with pytest.raises(ValueError):
      radar_vin.tesla_radar_security_key(invalid_seed)


def test_tesla_preap_safety_flags_are_stable():
  from opendbc.car.tesla.preap.safety_flags import TeslaPreAPSafetyFlags

  assert TeslaPreAPSafetyFlags.ENABLE_PEDAL == 1
  assert TeslaPreAPSafetyFlags.RADAR_EMULATION == 2
  assert TeslaPreAPSafetyFlags.RADAR_BEHIND_NOSECONE == 4
  assert TeslaPreAPSafetyFlags.RADAR_VIN_LEARN == 8


def test_radar_vin_assembler_reconstructs_out_of_order_fragments():
  radar_vin = _module()
  assembler = radar_vin.RadarVinAssembler()
  packets = [
    _can(0x405, b"\x11" + TARGET_VIN[3:10].encode(), 0),
    _can(0x405, b"\x12" + TARGET_VIN[10:17].encode(), 0),
    _can(0x405, b"\x10\x00\x00\x00\x00" + TARGET_VIN[:3].encode(), 0),
  ]

  assert assembler.update(packets, 10.0) == TARGET_VIN


def test_radar_vin_assembler_accepts_fragments_at_the_one_second_boundary():
  radar_vin = _module()
  assembler = radar_vin.RadarVinAssembler()
  packets = [
    _can(0x405, b"\x10\x00\x00\x00\x00" + TARGET_VIN[:3].encode(), 0),
    _can(0x405, b"\x11" + TARGET_VIN[3:10].encode(), 0),
    _can(0x405, b"\x12" + TARGET_VIN[10:].encode(), 0),
  ]

  assembler.update(packets, 1.0)
  assert assembler.update([], 2.0) == TARGET_VIN


def test_radar_vin_assembler_rejects_short_raw_fragment():
  radar_vin = _module()
  assembler = radar_vin.RadarVinAssembler()
  packets = [
    CanData(0x405, b"\x10\x00\x00\x00\x00AB", 0),
    _can(0x405, b"\x11" + TARGET_VIN[3:10].encode(), 0),
    _can(0x405, b"\x12" + TARGET_VIN[10:].encode(), 0),
  ]

  assert assembler.update(packets, 1.0) is None


@pytest.mark.parametrize(
  "invalid_frame",
  (
    CanData(0x405, b"\x10\x00\x00\x00ABC", 0),
    CanData(0x405, b"\x10\x00\x00\x00\x00ABC\x00\x00", 0),
  ),
)
def test_radar_vin_assembler_rejects_non_classical_can_frame_lengths(invalid_frame):
  radar_vin = _module()
  assembler = radar_vin.RadarVinAssembler()
  packets = [
    invalid_frame,
    _can(0x405, b"\x11" + TARGET_VIN[3:10].encode(), 0),
    _can(0x405, b"\x12" + TARGET_VIN[10:].encode(), 0),
  ]

  assert assembler.update(packets, 1.0) is None


@pytest.mark.parametrize(
  "packets,now",
  (
    ([_can(0x405, b"\x10\x00\x00\x00\x00ABC", 0)], 1.0),
    ([_can(0x405, b"\x10\x00\x00\x00\x00ABC", 1), _can(0x405, b"\x11DEFGHJK", 0), _can(0x405, b"\x12LMNPRST", 0)], 1.0),
    ([_can(0x404, b"\x10\x00\x00\x00\x00ABC", 0), _can(0x405, b"\x11DEFGHJK", 0), _can(0x405, b"\x12LMNPRST", 0)], 1.0),
    ([_can(0x405, b"\x10\x00\x00\x00\x00ABC", 1), _can(0x405, b"\x11DEFGHJK", 1), _can(0x405, b"\x12LMNPRST", 1)], 2.01),
    ([_can(0x405, b"\x10\x00\x00\x00\x00ABC", 0), _can(0x405, b"\x11DEFGHJK", 0), _can(0x405, b"\x12LMNPRS", 0)], 1.0),
  ),
)
def test_radar_vin_assembler_rejects_incomplete_stale_or_wrong_source(packets, now):
  radar_vin = _module()
  assert radar_vin.RadarVinAssembler().update(packets, now) is None


@pytest.mark.parametrize("invalid_vin", ("ABCDEFGHJKLMNPRSI", "abcdefghjklmnprst", "ABCDEFGHJKLMNPRS*"))
def test_radar_vin_assembler_rejects_illegal_vin_characters(invalid_vin):
  radar_vin = _module()
  assembler = radar_vin.RadarVinAssembler()
  packets = [
    _can(0x405, b"\x10\x00\x00\x00\x00" + invalid_vin[:3].encode(), 0),
    _can(0x405, b"\x11" + invalid_vin[3:10].encode(), 0),
    _can(0x405, b"\x12" + invalid_vin[10:].encode(), 0),
  ]

  assert assembler.update(packets, 1.0) is None


def test_learner_uses_exact_sequence_and_variable_length_positive_responses():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  assert _tx_payload(learner.update([], now)) == b"\x02\x27\x11\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)

  key_request = _tx_payload(learner.update([], now))
  assert key_request[:3] == b"\x06\x27\x12"
  assert key_request[-1:] == b"\x00"
  now = _respond(learner, b"\x67\x12", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x04\x31\x01\x0a\x03\x00\x00\x00"
  now = _respond(learner, b"\x71\x01\x0a\x03\x99", now + 0.01)

  assert learner.update([], now + 1.98).can_sends == ()
  now += 2.01
  assert _tx_payload(learner.update([], now)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"
  now = _respond(learner, b"\x71\x02\x0a\x03\x01", now + 0.01)

  now += 2.0
  assert _tx_payload(learner.update([], now)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"
  now = _respond(learner, b"\x71\x02\x0a\x03\x02", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x04\x31\x03\x0a\x03\x00\x00\x00"
  now = _respond(learner, b"\x71\x03\x0a\x03\x03", now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x03\x22\xf1\x90\x00\x00\x00\x00"
  now = _respond(learner, b"\x62\xf1\x90" + TARGET_VIN.encode(), now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"
  output = learner.update([_rx_single(b"\x50\x01\x00\x32\x01\xf4")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.COMPLETE
  assert output.result == radar_vin.RadarVinLearnerResult.LEARNED


def test_learner_reassembles_f190_across_updates_and_sends_flow_control_once():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  payload = b"\x62\xf1\x90" + OTHER_VIN.encode()

  output = learner.update([_rx_first(payload)], now + 0.01)
  assert output.can_sends == (_can(RADAR_TX_ADDRESS, b"\x30\x00\x00\x00\x00\x00\x00\x00"),)
  assert learner.update([], now + 0.02).can_sends == ()

  assert learner.update([_rx_consecutive(1, payload[6:13])], now + 0.03).can_sends == ()
  assert learner.update([_rx_consecutive(2, payload[13:])], now + 0.04).can_sends == ()
  assert _tx_payload(learner.update([], now + 0.05)) == b"\x02\x27\x11\x00\x00\x00\x00\x00"


def test_learner_flow_controls_phase_valid_multiframe_routine_response():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  assert _tx_payload(learner.update([], now)) == b"\x02\x27\x11\x00\x00\x00\x00\x00"
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x12", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)
  now += 2.0
  learner.update([], now)

  payload = b"\x71\x02\x0a\x03\x01\x02\x03\x04\x05"
  output = learner.update([_rx_first(payload)], now + 0.01)
  assert output.can_sends == (_can(RADAR_TX_ADDRESS, b"\x30\x00\x00\x00\x00\x00\x00\x00"),)
  assert learner.update([_rx_consecutive(1, payload[6:])], now + 0.02).can_sends == ()


def test_learner_advances_after_padded_final_routine_consecutive_frame():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_start_routine(learner)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)
  now += 2.0
  learner.update([], now)

  output = learner.update([_can(RADAR_RX_ADDRESS, b"\x10\x09\x71\x02\x0a\x03\x01\x02")], now + 0.01)
  assert output.can_sends == (_can(RADAR_TX_ADDRESS, b"\x30\x00\x00\x00\x00\x00\x00\x00"),)
  output = learner.update([_can(RADAR_RX_ADDRESS, b"\x21\x03\x04\x05\x00\x00\x00\x00")], now + 0.02)
  assert output.state == radar_vin.RadarVinLearnerState.WAIT_STOP
  assert _tx_payload(learner.update([], now + 2.03)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"


def test_learner_returns_already_matched_after_cleanup_without_security_access():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + TARGET_VIN.encode(), now + 0.01)

  assert _tx_payload(learner.update([], now)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"
  output = learner.update([_rx_single(b"\x50\x01")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.COMPLETE
  assert output.result == radar_vin.RadarVinLearnerResult.ALREADY_MATCHED


def test_learner_retries_only_negative_stop_responses_with_three_attempt_bound():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x12", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)

  for attempt in range(3):
    now += 2.0
    assert _tx_payload(learner.update([], now)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"
    output = learner.update([_rx_single(b"\x7f\x31\x22")], now + 0.01)
    now += 0.01
    if attempt < 2:
      assert output.state == radar_vin.RadarVinLearnerState.WAIT_STOP
    else:
      assert output.state == radar_vin.RadarVinLearnerState.CLEANUP


def test_learner_fails_closed_for_bad_seed_without_sending_key():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  learner.update([], now)

  output = learner.update([_rx_single(b"\x67\x11\x00\x01\x02")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert _tx_payload(learner.update([], now + 0.02)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"


def test_learner_does_not_retry_key_rejection():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_key_request(learner)

  output = learner.update([_rx_single(b"\x7f\x27\x35")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.KEY_REJECTED
  assert _tx_payload(learner.update([], now + 0.02)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"


def test_learner_fails_closed_for_invalid_pre_learn_vin():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  payload = b"\x62\xf1\x90" + b"ABCDEFGHJKLMNPRS*"

  learner.update([_rx_first(payload), _rx_consecutive(1, payload[6:13])], now + 0.01)
  output = learner.update([_rx_consecutive(2, payload[13:])], now + 0.02)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.MALFORMED_RESPONSE


def test_learner_delays_stop_retry_from_the_negative_response_time():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x12", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)

  delayed_stop_at = now + 10.0
  learner.update([], delayed_stop_at)
  learner.update([_rx_single(b"\x7f\x31\x22")], delayed_stop_at + 0.01)
  assert learner.update([], delayed_stop_at + 2.0).can_sends == ()
  assert _tx_payload(learner.update([], delayed_stop_at + 2.01)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"


def test_learner_gives_each_successful_stop_stage_its_own_retry_budget():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + OTHER_VIN.encode(), now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x11\x00\x09\xa5\xe1", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x67\x12", now + 0.01)
  learner.update([], now)
  now = _respond(learner, b"\x71\x01\x0a\x03", now + 0.01)

  now += 2.0
  learner.update([], now)
  learner.update([_rx_single(b"\x7f\x31\x22")], now + 0.01)
  now += 2.01
  learner.update([], now)
  now = _respond(learner, b"\x71\x02\x0a\x03", now + 0.01)

  for _ in range(2):
    now += 2.0
    learner.update([], now)
    output = learner.update([_rx_single(b"\x7f\x31\x22")], now + 0.01)
    assert output.state == radar_vin.RadarVinLearnerState.WAIT_STOP
    now += 0.01

  now += 2.0
  assert _tx_payload(learner.update([], now)) == b"\x04\x31\x02\x0a\x03\x00\x00\x00"


@pytest.mark.parametrize(
  "response",
  (
    b"\x63\xf1\x90" + OTHER_VIN.encode(),
    b"\x62\xf1\x91" + OTHER_VIN.encode(),
    b"\x62\xf1\x90" + OTHER_VIN.encode()[:-1],
  ),
)
def test_learner_fails_closed_for_unexpected_or_malformed_did_response(response):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  output = learner.update([_rx_single(response)], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure in (radar_vin.RadarVinFailure.MALFORMED_RESPONSE, radar_vin.RadarVinFailure.UNEXPECTED_RESPONSE)


@pytest.mark.parametrize(
  "response",
  (
    _can(RADAR_RX_ADDRESS + 1, b"\x03\x62\xf1\x90"),
    _can(RADAR_RX_ADDRESS, b"\x03\x62\xf1\x90", RADAR_BUS + 1),
  ),
)
def test_learner_ignores_responses_from_the_wrong_endpoint(response):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  output = learner.update([response], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.PRE_LEARN_VIN
  assert output.can_sends == ()


@pytest.mark.parametrize("response", (b"\x71\x02\x0a\x03", b"\x71\x01\x0a\x04"))
def test_learner_rejects_wrong_routine_positive_prefix(response):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_start_routine(learner)

  output = learner.update([_rx_single(response)], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.UNEXPECTED_RESPONSE


@pytest.mark.parametrize("response", (b"\x7f\x22", b"\x7f\x27\x22"))
def test_learner_rejects_malformed_negative_response(response):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  output = learner.update([_rx_single(response)], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.MALFORMED_RESPONSE


def test_learner_holds_on_response_pending_then_times_out_and_cleans_up():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  output = learner.update([_rx_single(b"\x7f\x22\x78")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.PRE_LEARN_VIN
  assert output.can_sends == ()

  output = learner.update([], now + 3.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.TIMEOUT


@pytest.mark.parametrize("response_kind", ("positive", "pending"))
def test_learner_enforces_original_deadline_before_late_responses(response_kind):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)

  if response_kind == "pending":
    learner.update([_rx_single(b"\x7f\x22\x78")], now + 1.0)
    late_frames = [_rx_single(b"\x7f\x22\x78")]
  else:
    payload = b"\x62\xf1\x90" + OTHER_VIN.encode()
    late_frames = [_rx_first(payload), _rx_consecutive(1, payload[6:13]), _rx_consecutive(2, payload[13:])]

  output = learner.update(late_frames, now + 3.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.TIMEOUT


def test_learner_readiness_polls_every_hundred_milliseconds_on_single_tick_updates():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  learner.start(TARGET_VIN, 0.0)
  learner.update([], 0.0)
  learner.update([_rx_single(b"\x7e\x00")], 0.01)
  learner.update([], 0.02)
  learner.update([_rx_single(b"\x50\x01")], 0.03)
  learner.update([], 0.04)
  learner.update([_rx_single(b"\x50\x03")], 0.05)

  ticks = list(range(6, 107))
  assert len(ticks) == len(set(ticks))
  readiness_polls = []
  for tick in ticks:
    now = tick / 100
    output = learner.update([], now)
    if output.can_sends:
      assert _tx_payload(output) == b"\x02\x3e\x00\x00\x00\x00\x00\x00"
      readiness_polls.append(tick)

  assert readiness_polls == list(range(6, 97, 10))
  assert learner.state == radar_vin.RadarVinLearnerState.CLEANUP


def test_learner_post_learn_mismatch_enters_cleanup():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_post_learn_vin(learner)

  output = learner.update([_rx_first(b"\x62\xf1\x90" + OTHER_VIN.encode()),
                           _rx_consecutive(1, OTHER_VIN.encode()[3:10]),
                           _rx_consecutive(2, OTHER_VIN.encode()[10:])], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.POST_LEARN_MISMATCH


def test_learner_cleanup_rejects_wrong_positive_response():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  now = _respond(learner, b"\x62\xf1\x90" + TARGET_VIN.encode(), now + 0.01)
  learner.update([], now)

  output = learner.update([_rx_single(b"\x50\x03")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.FAILED
  assert output.result == radar_vin.RadarVinLearnerResult.FAILED


def test_learner_discards_stale_cleanup_response_delivered_before_cleanup_request():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  now = _prepare_pre_learn_vin(learner)
  payload = b"\x62\xf1\x90" + TARGET_VIN.encode()

  output = learner.update([_rx_first(payload), _rx_consecutive(1, payload[6:13]), _rx_consecutive(2, payload[13:]),
                           _rx_single(b"\x50\x01")], now + 0.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert _tx_payload(learner.update([], now + 0.02)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"
  output = learner.update([], now + 3.03)
  assert output.state == radar_vin.RadarVinLearnerState.FAILED


def test_learner_update_documents_newly_received_chronological_frames():
  radar_vin = _module()
  docstring = radar_vin.RadarVinLearner.update.__doc__

  assert docstring is not None
  assert "newly received" in docstring
  assert "chronological" in docstring


def test_learner_overall_timeout_and_abort_enter_cleanup_then_fail_if_cleanup_times_out():
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  learner.start(TARGET_VIN, 0.0)
  learner.update([], 0.0)
  learner.update([_rx_single(b"\x7e\x00")], 0.01)

  output = learner.update([], 30.01)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert learner.failure == radar_vin.RadarVinFailure.OVERALL_TIMEOUT
  assert _tx_payload(learner.update([], 30.02)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"
  output = learner.update([], 33.03)
  assert output.state == radar_vin.RadarVinLearnerState.FAILED
  assert output.result == radar_vin.RadarVinLearnerResult.FAILED

  learner = radar_vin.RadarVinLearner()
  learner.start(TARGET_VIN, 0.0)
  learner.update([], 0.0)
  learner.update([_rx_single(b"\x7e\x00")], 0.01)
  output = learner.abort(radar_vin.RadarVinFailure.ABORTED, 0.02)
  assert output.state == radar_vin.RadarVinLearnerState.CLEANUP
  assert _tx_payload(learner.update([], 0.03)) == b"\x02\x10\x01\x00\x00\x00\x00\x00"


def test_learner_does_not_log_sensitive_protocol_values(caplog):
  radar_vin = _module()
  learner = radar_vin.RadarVinLearner()
  caplog.set_level(logging.DEBUG)

  now = _prepare_pre_learn_vin(learner)
  learner.update([_rx_single(b"\x62\xf1\x90" + OTHER_VIN.encode())], now + 0.01)
  learner.update([], now + 0.02)
  learner.update([_rx_single(b"\x67\x11\x00\x09\xa5\xe1")], now + 0.03)

  logged = caplog.text
  assert TARGET_VIN not in logged
  assert OTHER_VIN not in logged
  assert "0009a5e1" not in logged
