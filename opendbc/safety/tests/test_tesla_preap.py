#!/usr/bin/env python3
import unittest

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.preap.safety_flags import TeslaPreAPSafetyFlags
from opendbc.car.tesla.values import CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

# Stalk lever positions from tesla_preap.h
STALK_FWD_CANCEL = 1
STALK_RWD_ENGAGE = 2

RADAR_DIAG_TX_ADDR = 0x641
RADAR_DIAG_RX_ADDR = 0x651
RADAR_DIAG_BUS = 1

TESTER_PRESENT = b"\x02\x3e\x00\x00\x00\x00\x00\x00"
DEFAULT_SESSION = b"\x02\x10\x01\x00\x00\x00\x00\x00"
EXTENDED_SESSION = b"\x02\x10\x03\x00\x00\x00\x00\x00"
REQUEST_SEED = b"\x02\x27\x11\x00\x00\x00\x00\x00"
SEND_KEY = b"\x06\x27\x12\x12\x34\x56\x78\x00"
START_ROUTINE = b"\x04\x31\x01\x0a\x03\x00\x00\x00"
STOP_ROUTINE = b"\x04\x31\x02\x0a\x03\x00\x00\x00"
REQUEST_RESULTS = b"\x04\x31\x03\x0a\x03\x00\x00\x00"
READ_VIN = b"\x03\x22\xf1\x90\x00\x00\x00\x00"
FLOW_CONTROL = b"\x30\x00\x00\x00\x00\x00\x00\x00"

SYNTHETIC_VIN_RESPONSE = b"\x62\xf1\x90" + b"A" * 17


def _fix_epas_checksum(msg):
  """Compute Tesla byte-sum checksum for EPAS_sysStatus (checksum at byte 7)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 7:
      chk += data[i]
  data[7] = chk & 0xFF
  return addr, bytes(data), bus


def _fix_das_checksum(msg):
  """Compute Tesla byte-sum checksum for DAS_steeringControl (checksum at byte 3)."""
  addr, data, bus = msg
  data = bytearray(data)
  chk = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i in range(len(data)):
    if i != 3:
      chk += data[i]
  data[3] = chk & 0xFF
  return addr, bytes(data), bus


def _get_preap_vm():
  """Get VehicleModel matching PREAP_STEERING_PARAMS in tesla_preap.h."""
  from opendbc.car.tesla.interface import CarInterface
  return VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))


class TeslaPreAPTestMixin(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  # Abstract base class — concrete subclasses (SteeringOnly, WithPedal) do the work.
  # __test__ = False prevents pytest from collecting this class directly (it still
  # gets collected via MRO without this, because CarSafetyTest is a TestCase).
  __test__ = False
  # Pre-AP has no relay and no bus 2 forwarding
  RELAY_MALFUNCTION_ADDRS = {}
  FWD_BUS_LOOKUP = {}
  FWD_BLACKLISTED_ADDRS = {}

  TX_MSGS = [
    [0x488, 0],  # DAS_steeringControl
    [0x2B9, 0],  # DAS_control
    [0x214, 0],  # EPB_epasControl
    [0x551, 0],  # Pedal bus 0
    [0x551, 2],  # Pedal bus 2
    [0x45,  0],  # STW_ACTN_RQ (stalk spoof)
    [0x3E9, 0],  # DAS_bodyControls (turn signal)
  ]

  STANDSTILL_THRESHOLD = 0.5 / 3.6  # 0.5 kph in m/s

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10
  LATERAL_FREQUENCY = 50  # Hz

  # Tesla uses VM-based limits, not breakpoint tables
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  GAS_PRESSED_THRESHOLD = 0  # DI_torque1 byte 6 != 0

  cnt_epas = 0
  cnt_angle_cmd = 0

  packer: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    self.VM = _get_preap_vm()
    self.packer = CANPackerSafety("tesla_preap")
    self.safety = libsafety_py.libsafety

  def _angle_cmd_msg(self, angle, state, increment_timer=True, bus=0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values,
                                           fix_checksum=_fix_das_checksum)

  def _angle_meas_msg(self, angle, hands_on_level=0, eac_status=1, eac_error_code=0):
    values = {
      "EPAS_internalSAS": angle,
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code,
      "EPAS_sysStatusCounter": self.cnt_epas % 16,
    }
    self.__class__.cnt_epas += 1
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values,
                                           fix_checksum=_fix_epas_checksum)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}  # m/s to kph
    return self.packer.make_can_msg_safety("ESP_B", 0, values)

  def _speed_msg_2(self, speed):
    return None  # Pre-AP has no second speed source

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    lever = STALK_RWD_ENGAGE if enable else STALK_FWD_CANCEL
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": lever})

  def _gear_msg(self, gear):
    return self.packer.make_can_msg_safety("DI_torque2", 0, {"DI_gear": gear})

  def _di_brake_msg(self, brake):
    values = {"DI_gear": 4, "DI_brakePedal": 1 if brake else 0}
    return self.packer.make_can_msg_safety("DI_torque2", 0, values)

  def _door_msg(self, door_fl=0, door_fr=0, door_rl=0, door_rr=0):
    values = {
      "DOOR_STATE_FL": door_fl,
      "DOOR_STATE_FR": door_fr,
      "DOOR_STATE_RL": door_rl,
      "DOOR_STATE_RR": door_rr,
    }
    return self.packer.make_can_msg_safety("GTW_carState", 0, values)

  def _engage_and_advance_timer(self):
    """Engage via stalk and advance timer past the 600ms echo filter window."""
    self._rx(self._pcm_status_msg(True))
    self.safety.set_timer(700000)

  # =====================================================================
  # Base class overrides for Pre-AP differences
  # =====================================================================

  def test_angle_cmd_when_enabled(self):
    # Tesla uses VM-based limits — test_lateral_accel_limit covers this
    pass

  def test_angle_cmd_when_disabled(self):
    self._rx(self._angle_meas_msg(0))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, 0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(100, 0)))

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_vehicle_moving(self):
    # Pre-AP uses: vehicle_moving = speed > (0.5f * KPH_TO_MS)
    # Due to float32 precision in the DBC factor (0.00999999978 vs 0.01),
    # exactly 0.5 kph may register as slightly above threshold. Use values
    # that are unambiguously below/above regardless of float precision.
    self.assertFalse(self.safety.get_vehicle_moving())
    self._rx(self._speed_msg(0))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 0.3 kph → clearly below 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 0.3}))
    self.assertFalse(self.safety.get_vehicle_moving())
    # 1.0 kph → clearly above 0.5 kph threshold
    self._rx(self.packer.make_can_msg_safety("ESP_B", 0, {"ESP_vehicleSpeed": 1.0}))
    self.assertTrue(self.safety.get_vehicle_moving())

  def test_prev_user_brake(self):
    # PRE-AP BRAKE ARCHITECTURE:
    # The panda keeps the framework's brake_pressed=false so generic brake
    # disengagement never kills lateral control. Separate raw-brake latches
    # still block enabled pedal commands at the TX hook.
    #
    # Python ORs DI_torque2.DI_brakePedal and BrakeMessage.driverBrakeStatus,
    # drops longitudinal while keeping cruise/lateral enabled, and suppresses
    # public CarState.brakePressed. Panda independently ORs the same two raw
    # sources so it closes the interval before Python observes BrakeMessage.
    #
    # Framework invariant: brake_pressed remains false; pedal authority does not.
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_brake_pressed_prev())
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_brake_pressed_prev())

  def test_allow_user_brake_at_zero_speed(self):
    # Brake does not clear controls_allowed because lateral remains available;
    # the separate pedal TX interlock is covered below.
    self._rx(self._speed_msg(0))
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_not_allow_user_brake_when_moving(self):
    # Brake while moving still leaves controls_allowed set for lateral. Python
    # drops longitudinal, while panda independently blocks enabled pedal TX.
    self._rx(self._user_brake_msg(True))
    self.safety.set_controls_allowed(True)
    self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    # Pre-AP uses a 600ms echo filter on stalk cancel. Advancing the timer
    # past the window is required for cancel to take effect.
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      if not engaged:
        self.safety.set_timer(700000)
        self._rx(self._pcm_status_msg(False))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())

  def test_disable_control_allowed_from_cruise(self):
    self._engage_and_advance_timer()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  # test_tx_hook_on_wrong_safety_mode: inherited from base class, no override needed.

  def test_body_controls_turn_indicator_allowed(self):
    # Valid turn-indicator requests (0-3) are allowed when controls_allowed.
    self.safety.set_controls_allowed(True)
    for turn in range(4):
      msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
              {"DAS_turnIndicatorRequest": turn})
      self.assertTrue(self._tx(msg), f"turn={turn} should be allowed")

  def test_body_controls_blocked_when_not_allowed(self):
    # Like other actuation, blocked when controls are not allowed.
    self.safety.set_controls_allowed(False)
    msg = self.packer.make_can_msg_safety("DAS_bodyControls", 0,
            {"DAS_turnIndicatorRequest": 1})
    self.assertFalse(self._tx(msg))

  # =====================================================================
  # Pre-AP specific safety tests
  # =====================================================================

  def test_gear_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._gear_msg(4))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_door_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._door_msg(door_fl=1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_steering_disengage_hands_on(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_steering_disengage_epas_error_codes(self):
    # EPAS error codes 6-9 with EAC_INHIBITED (status=0) should disengage.
    # Must reinit safety between iterations to clear stale state.
    for error_code in [6, 7, 8, 9]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed(), f"Setup failed for error code {error_code}")
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertFalse(self.safety.get_controls_allowed(), f"Error code {error_code} should disengage")

  def test_steering_no_disengage_on_other_error_codes(self):
    for error_code in [0, 1, 2, 3, 4, 5, 10, 11, 12]:
      self.setUp()
      self._setup_safety_hooks()
      self._rx(self._pcm_status_msg(True))
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=0, eac_error_code=error_code))
      self.assertTrue(self.safety.get_controls_allowed(), f"Error code {error_code} should NOT disengage")

  def test_stalk_cancel_echo_filter(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel within echo window should be filtered
    self._rx(self._pcm_status_msg(False))
    self.assertTrue(self.safety.get_controls_allowed())
    # Cancel after echo window should work
    self.safety.set_timer(700000)
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stalk_rearm_after_steering_disengage(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._angle_meas_msg(0, hands_on_level=2))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_cruise_engaged_prev())
    self._rx(self._angle_meas_msg(0, hands_on_level=0))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_steering_control_type(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._angle_meas_msg(0))
    for control_type in range(4):
      should_tx = control_type in (0, 1)
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, control_type)),
                       f"Control type {control_type} should {'pass' if should_tx else 'block'}")

  def test_epas_control_type(self):
    for controls_allowed in (False, True):
      for mode in range(8):
        with self.subTest(controls_allowed=controls_allowed, mode=mode):
          self.safety.set_controls_allowed(controls_allowed)
          msg = self.packer.make_can_msg_safety("EPB_epasControl", 0, {"EPB_epasEACAllow": mode})
          should_tx = mode == 0 or (controls_allowed and mode == 1)
          self.assertEqual(should_tx, self._tx(msg),
                           f"EPB mode {mode} should {'pass' if should_tx else 'block'}")

  def test_no_aeb(self):
    self.safety.set_controls_allowed(True)
    for aeb_event in range(4):
      msg = self.packer.make_can_msg_safety("DAS_control", 0, {"DAS_aebEvent": aeb_event})
      should_tx = aeb_event == 0
      self.assertEqual(should_tx, self._tx(msg),
                       f"AEB event {aeb_event} should {'pass' if should_tx else 'block'}")

  def test_lateral_accel_limit(self):
    # Verify VM-based lateral accel limits constrain steering at speed.
    # Ramp steering angle up in max_delta increments at a fixed speed, find the
    # angle at which the panda blocks further increases, and assert it's within
    # a tight tolerance of Python's VehicleModel computation.
    #
    # Float precision note: panda uses float32 for the curvature_factor
    # computation while Python uses float64. The difference is typically < 2°
    # at highway speeds. We allow 25% tolerance to absorb this while still
    # catching any bug that would make the limit off by a factor of 2 or more
    # (e.g. wrong slip_factor, wrong MAX_LATERAL_ACCEL, wrong wheelbase).
    #
    # TODO: for bit-exact boundary testing, port the approach from
    # test_tesla_hw1.py (round_angle + _reset_speed_measurement +
    # set_desired_angle_last) which does precise +1/+2 CAN-unit tests by
    # matching the panda's float32 arithmetic in Python.
    for speed in [20.0, 30.0]:
      self.setUp()
      self._setup_safety_hooks()
      self.safety.set_controls_allowed(True)
      # Must fill the vehicle_speed sample buffer (6 slots) so min converges
      for _ in range(10):
        self._rx(self._speed_msg(speed))
      self._rx(self._angle_meas_msg(0))
      self._tx(self._angle_cmd_msg(0, 1))

      expected_max = get_max_angle_vm(speed, self.VM, CarControllerParams)
      max_delta = get_max_angle_delta_vm(max(speed, 1), self.VM, CarControllerParams)
      angle = 0.0
      blocked_at = None
      for _ in range(5000):
        next_angle = angle + max_delta
        if next_angle > self.STEER_ANGLE_MAX:
          break
        if not self._tx(self._angle_cmd_msg(next_angle, 1)):
          blocked_at = next_angle
          break
        angle = next_angle

      self.assertIsNotNone(
        blocked_at,
        f"Speed {speed}: VM limit never blocked — reached {angle:.1f} deg (Python expected max {expected_max:.1f} deg)",
      )
      # Tight bound: blocked angle must be within ±25% of Python's computation.
      # Absorbs float32/float64 drift but catches order-of-magnitude bugs.
      lower_bound = expected_max * 0.75
      upper_bound = expected_max * 1.25
      self.assertGreaterEqual(
        blocked_at,
        lower_bound,
        f"Speed {speed}: blocked at {blocked_at:.1f} deg — too LOW (expected ~{expected_max:.1f}, bound {lower_bound:.1f})",
      )
      self.assertLessEqual(
        blocked_at,
        upper_bound,
        f"Speed {speed}: blocked at {blocked_at:.1f} deg — too HIGH (expected ~{expected_max:.1f}, bound {upper_bound:.1f})",
      )

  def _setup_safety_hooks(self):
    """Subclasses call this to set up the correct safety hooks."""
    raise NotImplementedError


class TestTeslaPreAPSteeringOnly(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with no pedal — lateral only."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()

  def test_pedal_blocked_without_flag(self):
    self.safety.set_controls_allowed(True)
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_no_pedal_does_not_invalidate_rx_checks(self):
    # Regression: route 02ae0a637825acd6|196fda2496 — tester with no Comma Pedal
    # hit "Controls Mismatch" because the 0x552 rx_check used frequency=0,
    # which divides by zero in safety_tick (safety.h:330) and marks it lagging.
    # When ENABLE_PEDAL is unset, 0x552 must not be in rx_checks at all.
    di_state = self.packer.make_can_msg_safety("DI_state", 0, {"DI_state": 1})
    for msg in (self._angle_meas_msg(0), self._pcm_status_msg(False), self._speed_msg(0),
                self._user_brake_msg(False), self._user_gas_msg(0), self._gear_msg(4),
                self._door_msg(), di_state):
      self._rx(msg)
    # Tick 500ms after msgs — under the 1s min lag window for real checks but long
    # enough that a freq=0 divide-by-zero would have already tripped the pedal row.
    self.safety.set_timer(int(5e5))
    self.safety.safety_tick_current_safety_config()
    self.assertTrue(self.safety.safety_config_valid(),
                    "rx_checks must stay valid without a pedal installed")


class TestTeslaPreAPWithPedal(TeslaPreAPTestMixin, unittest.TestCase):
  """Pre-AP with Comma Pedal enabled."""
  __test__ = True  # re-enable collection (mixin sets __test__=False)

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(TeslaPreAPSafetyFlags.ENABLE_PEDAL))
    self.safety.init_tests()

  # Pedal interceptor (0x552) values are raw 16-bit integers read by the panda as
  # `(data[0] << 8) | data[1]`. The DBC scales them to physical:
  #   physical = raw * 0.0507968128 - 22.85856576
  # Panda threshold: raw > 650 → gas_pressed (chosen from real drive data; see
  # comments in tesla_preap.h rx_hook). Helper values below:
  PEDAL_RAW_AT_REST_MAX = 633      # max observed at rest in real drive data
  PEDAL_RAW_NOISE_THRESHOLD = 650  # panda threshold
  PEDAL_RAW_CLEAR_PRESS = 800      # clearly pressed

  @staticmethod
  def _raw_to_physical(raw):
    return raw * 0.0507968128 - 22.85856576

  def _pedal_msg(self, raw_value, bus=0):
    """Craft a 0x552 message with the given raw value by converting to physical."""
    return self.packer.make_can_msg_safety("GAS_SENSOR", bus,
                                           {"INTERCEPTOR_GAS": self._raw_to_physical(raw_value)})

  def _user_gas_msg(self, gas):
    # With pedal enabled, gas is detected from pedal interceptor (0x552),
    # not DI_torque1. The C code ignores DI_torque1 gas when pedal is active.
    # Use clearly pressed value when gas=True; clearly not pressed when gas=False.
    raw = self.PEDAL_RAW_CLEAR_PRESS if gas else 400
    return self._pedal_msg(raw)

  def test_pedal_allowed_with_flag(self):
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertTrue(self._tx(msg))

  def test_pedal_blocked_without_controls(self):
    self.assertFalse(self.safety.get_controls_allowed())
    msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(msg))

  def test_pedal_gas_detection_bus_0(self):
    # Verify pedal gas detection works on bus 0 (first wiring config).
    self.assertFalse(self.safety.get_gas_pressed_prev())
    # Clearly pressed: raw 800 → > 650 → gas_pressed=True
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 0 must set gas_pressed")
    # Clearly not pressed: raw 400 → < 650 → gas_pressed=False
    self._rx(self._pedal_msg(400, bus=0))
    self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_pedal_gas_detection_bus_2(self):
    # Verify pedal gas detection works on bus 2 (second wiring config).
    # Regression test: earlier version had `if (msg->bus != 0U) return;` at the
    # top of rx_hook that broke bus-2-wired pedals.
    self.assertFalse(self.safety.get_gas_pressed_prev())
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev(),
                    "Pedal gas on bus 2 must set gas_pressed (wiring config variant)")

  def test_pedal_gas_blocks_longitudinal_tx(self):
    # Full-chain test: pedal press → gas_pressed → !get_longitudinal_allowed() → pedal TX blocked.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_longitudinal_allowed())
    # Press pedal (clearly pressed)
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=0))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    # Pedal TX must be blocked
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    self.assertFalse(self._tx(tx_msg), "Pedal TX must be blocked during gas press")

  def test_pedal_rest_noise_does_not_trigger_gas(self):
    # Regression test for the pedal-engagement bug found in drive d0cdc986c5d023f5.
    # The pedal interceptor's resting voltage oscillates with noise; real Pre-AP
    # drive data showed raw values 424-633 while the driver was NOT pressing gas.
    # The original threshold of 450 was inside this noise range, causing false
    # gas_pressed readings that blocked pedal TX and prevented engagement.
    #
    # Verify that values across the entire observed rest-noise range do NOT
    # trigger gas_pressed.
    for raw in [424, 450, 475, 500, 550, 600, self.PEDAL_RAW_AT_REST_MAX]:
      for bus in [0, 2]:
        self.setUp()
        self._setup_safety_hooks()
        self._rx(self._pedal_msg(raw, bus=bus))
        self.assertFalse(self.safety.get_gas_pressed_prev(),
                         f"Raw {raw} on bus {bus} must NOT trigger gas_pressed (in rest noise range)")

  def test_pedal_rest_noise_does_not_block_longitudinal(self):
    # End-to-end regression test: after engaging, pedal rest noise must not cause
    # longitudinal TX to be blocked. Before this fix, noise-level raw values
    # (450-633) were stuck setting gas_pressed=True, blocking all pedal TX.
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    tx_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
    # Pump pedal messages across the at-rest noise range; TX must remain allowed
    for raw in [424, 450, 500, 550, 600, 633]:
      self._rx(self._pedal_msg(raw, bus=2))  # real drive had pedal on bus 2
      self.assertFalse(self.safety.get_gas_pressed_prev(),
                       f"Raw {raw} (noise) must not set gas_pressed")
      self.assertTrue(self._tx(tx_msg),
                      f"Pedal TX must be allowed at raw {raw} (noise range)")

  def test_pedal_release_enable_0_always_allowed(self):
    # A disabled GAS_COMMAND relinquishes authority and lets Comma Pedal pass
    # the driver's OEM pedal voltage through. The controller sends it once on
    # authority loss; panda must allow that release regardless of engagement.
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})

    # Case 1: not engaged, no gas — still allowed (benign)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed when not engaged")

    # Case 2: driver gas must not prevent the one-shot authority release.
    self._rx(self._pcm_status_msg(True))
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertTrue(self.safety.get_gas_pressed_prev())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertTrue(self._tx(disable_msg),
                    "enable=0 must be allowed during gas override")

  def test_pedal_enable_1_blocked_on_gas_press(self):
    # Conversely, enable=1 (authoritative accel command) MUST be blocked
    # when driver is pressing gas, preventing openpilot from overriding the driver.
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(enable_msg), "enable=1 allowed before gas press")

    # Driver presses gas
    self._rx(self._pedal_msg(self.PEDAL_RAW_CLEAR_PRESS, bus=2))
    self.assertFalse(self._tx(enable_msg),
                     "enable=1 must be blocked during driver gas press")

  def test_each_raw_brake_source_blocks_only_enabled_pedal_commands(self):
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})

    for source, brake_msg in (("DI_torque2", self._di_brake_msg(True)),
                              ("BrakeMessage", self._user_brake_msg(True))):
      with self.subTest(source=source):
        self.setUp()
        self._rx(self._pcm_status_msg(True))
        self.assertTrue(self._tx(enable_msg))

        self._rx(brake_msg)

        self.assertTrue(self.safety.get_controls_allowed())
        self.assertFalse(self._tx(enable_msg))
        self.assertTrue(self._tx(disable_msg))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))

  def test_di_brake_closes_twenty_ms_seam_and_sources_clear_independently(self):
    enable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 0, "ENABLE": 1})
    self._rx(self._pcm_status_msg(True))
    self._rx(self._di_brake_msg(False))
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

    self._rx(self._di_brake_msg(True))
    self.assertFalse(self._tx(enable_msg))

    self.safety.set_timer(20000)
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self._tx(enable_msg))

    self._rx(self._di_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

    self._rx(self._user_brake_msg(True))
    self.assertFalse(self._tx(enable_msg))
    self._rx(self._di_brake_msg(False))
    self.assertFalse(self._tx(enable_msg))

    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._tx(enable_msg))

  def test_pedal_enable_0_blocked_without_flag(self):
    # If PREAP_FLAG_ENABLE_PEDAL is not set, NO 0x551 TX is allowed
    # (not even enable=0). This is the "pedal feature disabled" gate.
    # Override setUp to init without the pedal flag.
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, 0)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    disable_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                  {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertFalse(self._tx(disable_msg),
                     "enable=0 must still be blocked without PREAP_FLAG_ENABLE_PEDAL")

  def test_pedal_enable_0_with_high_gas_blocked(self):
    # Defense-in-depth: ENABLE=0 + non-zero GAS_COMMAND must be blocked.
    # Legitimate passthrough sends GAS_COMMAND=0 (physical) which is raw ~450.
    # Any ENABLE=0 message with a raw value above 500 is suspicious (possible
    # bug or attack attempting to exploit a hypothetical Comma Pedal firmware
    # flaw where ENABLE=0 is not honored).
    # Verify: legitimate passthrough (physical 0) allowed, high-value blocked.
    self._rx(self._pcm_status_msg(True))  # engage
    # Legitimate: physical 0 = raw 450 → <=500 → allowed
    ok_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                             {"GAS_COMMAND": 0, "ENABLE": 0})
    self.assertTrue(self._tx(ok_msg))
    # Attack: physical 100 = raw 2419 → >500 → blocked
    attack_msg = self.packer.make_can_msg_safety("GAS_COMMAND", 0,
                                                 {"GAS_COMMAND": 100, "ENABLE": 0})
    self.assertFalse(self._tx(attack_msg),
                     "ENABLE=0 with high GAS_COMMAND must be blocked (defense-in-depth)")


class TeslaPreAPRadarVinTestMixin(TeslaPreAPTestMixin):
  __test__ = False

  REQUEST_STAGES = (
    ("tester", TESTER_PRESENT, frozenset()),
    ("default", DEFAULT_SESSION, frozenset()),
    ("extended", EXTENDED_SESSION, frozenset()),
    ("readiness", TESTER_PRESENT, frozenset()),
    ("read_vin", READ_VIN, frozenset()),
    ("seed", REQUEST_SEED, frozenset()),
    ("key", SEND_KEY, frozenset((3, 4, 5, 6))),
    ("start", START_ROUTINE, frozenset()),
    ("stop", STOP_ROUTINE, frozenset()),
    ("results", REQUEST_RESULTS, frozenset()),
    ("final_vin", READ_VIN, frozenset()),
    ("cleanup", DEFAULT_SESSION, frozenset()),
  )

  AWAIT_CASES = (
    ("tester", TESTER_PRESENT, 0x3E, b"\x7e\x00", DEFAULT_SESSION),
    ("default", DEFAULT_SESSION, 0x10, b"\x50\x01", EXTENDED_SESSION),
    ("extended", EXTENDED_SESSION, 0x10, b"\x50\x03", TESTER_PRESENT),
    ("readiness", TESTER_PRESENT, 0x3E, b"\x7e\x00", READ_VIN),
    ("read_vin", READ_VIN, 0x22, SYNTHETIC_VIN_RESPONSE, REQUEST_SEED),
    ("seed", REQUEST_SEED, 0x27, b"\x67\x11\x01\x02\x03\x04", SEND_KEY),
    ("key", SEND_KEY, 0x27, b"\x67\x12", START_ROUTINE),
    ("start", START_ROUTINE, 0x31, b"\x71\x01\x0a\x03", STOP_ROUTINE),
    ("stop", STOP_ROUTINE, 0x31, b"\x71\x02\x0a\x03", STOP_ROUTINE),
    ("results", REQUEST_RESULTS, 0x31, b"\x71\x03\x0a\x03", READ_VIN),
    ("final_vin", READ_VIN, 0x22, SYNTHETIC_VIN_RESPONSE, DEFAULT_SESSION),
    ("cleanup", DEFAULT_SESSION, 0x10, b"\x50\x01", None),
  )

  def _setup_safety_hooks(self):
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(TeslaPreAPSafetyFlags.RADAR_VIN_LEARN))
    self.safety.init_tests()

  def _diag_tx(self, data, addr=RADAR_DIAG_TX_ADDR, bus=RADAR_DIAG_BUS):
    return self._tx(libsafety_py.make_CANPacket(addr, bus, data))

  def _diag_rx(self, data, addr=RADAR_DIAG_RX_ADDR, bus=RADAR_DIAG_BUS):
    return self._rx(libsafety_py.make_CANPacket(addr, bus, data))

  def _gtw_emissions(self):
    emissions = []
    count = self.safety.get_tesla_preap_gtw_debug_count()
    for index in range(count):
      packet = libsafety_py.ffi.new("CANPacket_t *")
      self.assertTrue(self.safety.get_tesla_preap_gtw_debug_packet(index, packet))
      emissions.append((packet.addr, packet.bus, bytes(packet.data[0:8])))
    return emissions

  def _single_response(self, payload):
    assert len(payload) <= 7
    self._diag_rx(bytes([len(payload)]) + payload + bytes(7 - len(payload)))

  @staticmethod
  def _first_frame(payload_prefix, declared_length):
    assert 7 < declared_length <= 0xFFF
    assert len(payload_prefix) <= 6
    return bytes([0x10 | (declared_length >> 8), declared_length & 0xFF]) + payload_prefix + bytes(6 - len(payload_prefix))

  def _multiframe_response(self, payload):
    assert 7 < len(payload) <= 0xFFF
    self._diag_rx(self._first_frame(payload[:6], len(payload)))
    self.assertTrue(self._diag_tx(FLOW_CONTROL))

    offset = 6
    sequence = 1
    while offset < len(payload):
      chunk = payload[offset:offset + 7]
      frame = bytes([0x20 | sequence]) + chunk + bytes(7 - len(chunk))
      self._diag_rx(frame)
      offset += len(chunk)
      sequence = (sequence + 1) & 0xF

  def _positive_response(self, payload):
    if len(payload) <= 7:
      self._single_response(payload)
    else:
      self._multiframe_response(payload)

  def _request_and_single_response(self, request, response):
    self.assertTrue(self._diag_tx(request))
    self._single_response(response)

  def _prepare_stage(self, stage):
    self._setup_safety_hooks()
    if stage == "tester":
      return

    self._request_and_single_response(TESTER_PRESENT, b"\x7e\x00")
    if stage == "default":
      return

    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")
    if stage == "extended":
      return

    self._request_and_single_response(EXTENDED_SESSION, b"\x50\x03")
    if stage == "readiness":
      return

    self._request_and_single_response(TESTER_PRESENT, b"\x7e\x00")
    if stage == "read_vin":
      return

    self.assertTrue(self._diag_tx(READ_VIN))
    self._multiframe_response(SYNTHETIC_VIN_RESPONSE)
    if stage == "seed":
      return

    self._request_and_single_response(REQUEST_SEED, b"\x67\x11\x01\x02\x03\x04")
    if stage == "key":
      return

    self._request_and_single_response(SEND_KEY, b"\x67\x12")
    if stage == "start":
      return

    self._request_and_single_response(START_ROUTINE, b"\x71\x01\x0a\x03")
    if stage == "stop":
      return

    self._request_and_single_response(STOP_ROUTINE, b"\x71\x02\x0a\x03")
    self._request_and_single_response(STOP_ROUTINE, b"\x71\x02\x0a\x03")
    if stage == "results":
      return

    self._request_and_single_response(REQUEST_RESULTS, b"\x71\x03\x0a\x03")
    if stage == "final_vin":
      return

    self.assertTrue(self._diag_tx(READ_VIN))
    self._multiframe_response(SYNTHETIC_VIN_RESPONSE)
    if stage == "cleanup":
      return

    raise AssertionError(f"unknown diagnostic stage: {stage}")

  def _epas_control_msg(self, mode):
    return self.packer.make_can_msg_safety("EPB_epasControl", 0, {"EPB_epasEACAllow": mode})

  def _das_control_msg(self):
    return self.packer.make_can_msg_safety("DAS_control", 0, {"DAS_aebEvent": 0})

  def _body_control_msg(self):
    return self.packer.make_can_msg_safety("DAS_bodyControls", 0, {"DAS_turnIndicatorRequest": 1})

  def _stalk_tx_msg(self):
    return self.packer.make_can_msg_safety("STW_ACTN_RQ", 0, {"SpdCtrlLvr_Stat": STALK_RWD_ENGAGE})

  def _enter_latched_state(self, state):
    self._setup_safety_hooks()
    if state == "active":
      self.assertTrue(self._diag_tx(TESTER_PRESENT))
    elif state == "poisoned":
      self.assertTrue(self._diag_tx(TESTER_PRESENT))
      self.assertFalse(self._diag_tx(REQUEST_SEED))
    elif state == "cleanup_pending":
      self.assertTrue(self._diag_tx(DEFAULT_SESSION))
    else:
      raise AssertionError(f"unknown latch state: {state}")

  def _assert_non_pedal_release_policy(self):
    self._rx(self._angle_meas_msg(0))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, 0)))
    self.assertTrue(self._tx(self._epas_control_msg(0)))

    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, 1)))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._epas_control_msg(1)))
    self.assertFalse(self._tx(self._das_control_msg()))
    self.assertFalse(self._tx(self._body_control_msg()))
    self.assertFalse(self._tx(self._stalk_tx_msg()))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_capability_whitelists_only_exact_diagnostic_address_bus_and_length(self):
    self.assertTrue(self._diag_tx(TESTER_PRESENT))

    for addr in (RADAR_DIAG_TX_ADDR - 1, RADAR_DIAG_TX_ADDR + 1):
      self._setup_safety_hooks()
      self.assertFalse(self._diag_tx(TESTER_PRESENT, addr=addr))
    for bus in (0, 2, 3):
      self._setup_safety_hooks()
      self.assertFalse(self._diag_tx(TESTER_PRESENT, bus=bus))
    for length in range(9):
      if length == 8:
        continue
      self._setup_safety_hooks()
      self.assertFalse(self._diag_tx(TESTER_PRESENT[:length]))

    for flags in (TeslaPreAPSafetyFlags(0), TeslaPreAPSafetyFlags.RADAR_EMULATION):
      self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(flags))
      self.safety.init_tests()
      for _, request, _ in self.REQUEST_STAGES:
        self.assertFalse(self._diag_tx(request))

  def test_exact_happy_path_and_normal_behavior_after_cleanup(self):
    self._prepare_stage("tester")
    self.assertFalse(self.safety.get_controls_allowed())
    self._request_and_single_response(TESTER_PRESENT, b"\x7e\x00")
    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01\x00\x32\x01\xf4")
    self._request_and_single_response(EXTENDED_SESSION, b"\x50\x03")
    self._request_and_single_response(TESTER_PRESENT, b"\x7e\x00")

    self.assertTrue(self._diag_tx(READ_VIN))
    self._multiframe_response(SYNTHETIC_VIN_RESPONSE)
    self._request_and_single_response(REQUEST_SEED, b"\x67\x11\x01\x02\x03\x04")
    self._request_and_single_response(SEND_KEY, b"\x67\x12")
    self._request_and_single_response(START_ROUTINE, b"\x71\x01\x0a\x03")
    self._request_and_single_response(STOP_ROUTINE, b"\x71\x02\x0a\x03")
    self._request_and_single_response(STOP_ROUTINE, b"\x71\x02\x0a\x03")
    self._request_and_single_response(REQUEST_RESULTS, b"\x71\x03\x0a\x03")
    self.assertTrue(self._diag_tx(READ_VIN))
    self._multiframe_response(SYNTHETIC_VIN_RESPONSE)
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))
    self._single_response(b"\x50\x01")

    self.assertFalse(self._diag_tx(REQUEST_SEED))
    self.safety.set_controls_allowed(True)
    self._rx(self._angle_meas_msg(0))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))
    self.assertTrue(self._tx(self._epas_control_msg(1)))
    self.assertTrue(self._tx(self._das_control_msg()))
    self.assertTrue(self._tx(self._body_control_msg()))
    self.assertFalse(self._diag_tx(DEFAULT_SESSION))

  def test_every_fixed_request_bit_is_required(self):
    for stage, request, wildcard_indexes in self.REQUEST_STAGES:
      for byte_index in range(len(request)):
        if byte_index in wildcard_indexes:
          continue
        for bit in range(8):
          self._prepare_stage(stage)
          mutated = bytearray(request)
          mutated[byte_index] ^= 1 << bit
          if bytes(mutated) == DEFAULT_SESSION:
            continue
          self.assertFalse(self._diag_tx(bytes(mutated)), (stage, byte_index, bit))

  def test_all_wrong_services_subfunctions_and_identifiers_are_rejected(self):
    fields = (
      ("tester", TESTER_PRESENT, 1, 0x3E),
      ("tester", TESTER_PRESENT, 2, 0x00),
      ("default", DEFAULT_SESSION, 1, 0x10),
      ("default", DEFAULT_SESSION, 2, 0x01),
      ("extended", EXTENDED_SESSION, 1, 0x10),
      ("extended", EXTENDED_SESSION, 2, 0x03),
      ("read_vin", READ_VIN, 1, 0x22),
      ("read_vin", READ_VIN, 2, 0xF1),
      ("read_vin", READ_VIN, 3, 0x90),
      ("seed", REQUEST_SEED, 1, 0x27),
      ("seed", REQUEST_SEED, 2, 0x11),
      ("key", SEND_KEY, 1, 0x27),
      ("key", SEND_KEY, 2, 0x12),
      ("start", START_ROUTINE, 1, 0x31),
      ("start", START_ROUTINE, 2, 0x01),
      ("start", START_ROUTINE, 3, 0x0A),
      ("start", START_ROUTINE, 4, 0x03),
      ("stop", STOP_ROUTINE, 1, 0x31),
      ("stop", STOP_ROUTINE, 2, 0x02),
      ("stop", STOP_ROUTINE, 3, 0x0A),
      ("stop", STOP_ROUTINE, 4, 0x03),
      ("results", REQUEST_RESULTS, 1, 0x31),
      ("results", REQUEST_RESULTS, 2, 0x03),
      ("results", REQUEST_RESULTS, 3, 0x0A),
      ("results", REQUEST_RESULTS, 4, 0x03),
    )
    for stage, request, byte_index, expected in fields:
      for value in range(256):
        if value == expected:
          continue
        self._prepare_stage(stage)
        mutated = bytearray(request)
        mutated[byte_index] = value
        if bytes(mutated) == DEFAULT_SESSION:
          continue
        self.assertFalse(self._diag_tx(bytes(mutated)), (stage, byte_index, value))

  def test_key_data_bytes_are_arbitrary_but_key_is_one_shot(self):
    self._prepare_stage("key")
    self.assertTrue(self._diag_tx(b"\x06\x27\x12\x00\xff\x5a\xa5\x00"))
    self.assertFalse(self._diag_tx(SEND_KEY))
    self._single_response(b"\x67\x12")
    self.assertFalse(self._diag_tx(SEND_KEY))

  def test_readiness_tester_present_can_be_polled_until_response(self):
    self._prepare_stage("readiness")
    for _ in range(10):
      self.assertTrue(self._diag_tx(TESTER_PRESENT))
      self.assertFalse(self.safety.get_controls_allowed())
    self._single_response(b"\x7e\x00")
    self.assertTrue(self._diag_tx(READ_VIN))

    self._prepare_stage("readiness")
    for _ in range(10):
      self.assertTrue(self._diag_tx(TESTER_PRESENT))
    self.assertFalse(self._diag_tx(TESTER_PRESENT))
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_out_of_order_requests_host_multiframe_and_flow_control_are_rejected(self):
    premature = (EXTENDED_SESSION, REQUEST_SEED, SEND_KEY, START_ROUTINE, STOP_ROUTINE,
                 REQUEST_RESULTS, READ_VIN, FLOW_CONTROL, b"\x10\x08\x31\x01\x0a\x03\x00\x00",
                 b"\x21\x00\x00\x00\x00\x00\x00\x00")
    for request in premature:
      with self.subTest(request=request):
        self._setup_safety_hooks()
        self.assertFalse(self._diag_tx(request))

    self._prepare_stage("stop")
    self._request_and_single_response(STOP_ROUTINE, b"\x71\x02\x0a\x03")
    self.assertFalse(self._diag_tx(REQUEST_RESULTS))

  def test_stop_retries_are_bounded_per_successful_stage(self):
    self._prepare_stage("stop")
    for _stage in range(2):
      for _ in range(2):
        self.assertTrue(self._diag_tx(STOP_ROUTINE))
        self._single_response(b"\x7f\x31\x22")
      self.assertTrue(self._diag_tx(STOP_ROUTINE))
      self._single_response(b"\x71\x02\x0a\x03")

    self.assertTrue(self._diag_tx(REQUEST_RESULTS))
    self.assertFalse(self._diag_tx(STOP_ROUTINE))

    self._prepare_stage("stop")
    for _ in range(3):
      self.assertTrue(self._diag_tx(STOP_ROUTINE))
      self._single_response(b"\x7f\x31\x22")
    self.assertFalse(self._diag_tx(STOP_ROUTINE))
    self.assertFalse(self._diag_tx(REQUEST_RESULTS))
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_response_pending_keeps_only_the_current_request_alive(self):
    self._prepare_stage("stop")
    self.assertTrue(self._diag_tx(STOP_ROUTINE))
    self._single_response(b"\x7f\x31\x78")
    self.assertFalse(self._diag_tx(STOP_ROUTINE))
    self.assertFalse(self._diag_tx(REQUEST_RESULTS))
    self._single_response(b"\x71\x02\x0a\x03")
    self.assertFalse(self._diag_tx(REQUEST_RESULTS))

  def test_matching_response_pending_keeps_every_await_phase_alive(self):
    for stage, request, request_sid, positive, next_request in self.AWAIT_CASES:
      with self.subTest(stage=stage):
        self._prepare_stage(stage)
        self.assertTrue(self._diag_tx(request))
        self.safety.set_timer(29999999)
        self._single_response(bytes([0x7F, request_sid, 0x78]))
        self.assertFalse(self.safety.get_controls_allowed())

        self.safety.set_timer(59999998)
        self._positive_response(positive)
        if stage == "tester":
          self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")
          self.assertTrue(self._diag_tx(EXTENDED_SESSION))
        elif stage == "cleanup":
          self.safety.set_controls_allowed(True)
          self._rx(self._angle_meas_msg(0))
          self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))
        else:
          self.assertTrue(self._diag_tx(next_request))

  def test_wrong_response_pending_sid_and_non_pending_nrc_poison(self):
    for stage, request, request_sid, positive, next_request in self.AWAIT_CASES:
      bad_responses = [bytes([0x7F, (request_sid + 1) & 0xFF, 0x78])]
      if stage != "stop":
        bad_responses.append(bytes([0x7F, request_sid, 0x22]))

      for bad_response in bad_responses:
        with self.subTest(stage=stage, response=bad_response):
          self._prepare_stage(stage)
          self.assertTrue(self._diag_tx(request))
          self._single_response(bad_response)

          if len(positive) > 7:
            self._diag_rx(self._first_frame(positive[:6], len(positive)))
            self.assertFalse(self._diag_tx(FLOW_CONTROL))
          elif stage == "tester":
            self._single_response(positive)
            self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")
            self.assertFalse(self._diag_tx(EXTENDED_SESSION))
          elif stage == "cleanup":
            self._single_response(positive)
            self.safety.set_controls_allowed(True)
            self._rx(self._angle_meas_msg(0))
            self.assertFalse(self._tx(self._angle_cmd_msg(0, 1)))
          else:
            self._single_response(positive)
            self.assertFalse(self._diag_tx(next_request))

          self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_completed_attempt_cannot_restart_before_safety_reinitialization(self):
    self._prepare_stage("cleanup")
    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")

    self.assertFalse(self._diag_tx(TESTER_PRESENT))
    self.assertFalse(self._diag_tx(REQUEST_SEED))
    self.assertFalse(self._diag_tx(SEND_KEY))
    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")
    self.assertFalse(self._diag_tx(TESTER_PRESENT))

    self._setup_safety_hooks()
    self.assertTrue(self._diag_tx(TESTER_PRESENT))

  def test_failed_key_attempt_cannot_restart_after_cleanup(self):
    self._prepare_stage("key")
    invalid_key = SEND_KEY[:-1] + b"\x01"
    self.assertFalse(self._diag_tx(invalid_key))
    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")

    self.assertFalse(self._diag_tx(TESTER_PRESENT))
    self.assertFalse(self._diag_tx(REQUEST_SEED))
    self.assertFalse(self._diag_tx(SEND_KEY))
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

    self._setup_safety_hooks()
    self.assertTrue(self._diag_tx(TESTER_PRESENT))

  def test_cleanup_only_bootstrap_does_not_consume_an_attempt(self):
    self._setup_safety_hooks()
    self._request_and_single_response(DEFAULT_SESSION, b"\x50\x01")
    self.assertTrue(self._diag_tx(TESTER_PRESENT))

  def test_multiframe_prefix_and_one_shot_flow_control(self):
    cases = (
      ("default", DEFAULT_SESSION, b"\x50\x01" + b"\x00" * 6, 2),
      ("extended", EXTENDED_SESSION, b"\x50\x03" + b"\x00" * 6, 2),
      ("start", START_ROUTINE, b"\x71\x01\x0a\x03" + b"\x00" * 4, 4),
      ("stop", STOP_ROUTINE, b"\x71\x02\x0a\x03" + b"\x00" * 4, 4),
      ("results", REQUEST_RESULTS, b"\x71\x03\x0a\x03" + b"\x00" * 4, 4),
      ("read_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE, 3),
      ("final_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE, 3),
      ("cleanup", DEFAULT_SESSION, b"\x50\x01" + b"\x00" * 6, 2),
    )
    for stage, request, response, prefix_length in cases:
      with self.subTest(stage=stage):
        self._prepare_stage(stage)
        self.assertTrue(self._diag_tx(request))
        self._diag_rx(self._first_frame(response[:6], len(response)))
        self.assertTrue(self._diag_tx(FLOW_CONTROL))
        self.assertFalse(self._diag_tx(FLOW_CONTROL))

      for prefix_index in range(prefix_length):
        with self.subTest(stage=stage, invalid_prefix=prefix_index):
          self._prepare_stage(stage)
          self.assertTrue(self._diag_tx(request))
          invalid = bytearray(response)
          invalid[prefix_index] ^= 1
          self._diag_rx(self._first_frame(bytes(invalid[:6]), len(invalid)))
          self.assertFalse(self._diag_tx(FLOW_CONTROL))
          self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_first_frame_length_is_validated_before_flow_control(self):
    cases = (
      ("tester", TESTER_PRESENT, b"\x7e\x00", 8),
      ("readiness", TESTER_PRESENT, b"\x7e\x00", 8),
      ("seed", REQUEST_SEED, b"\x67\x11\x01\x02\x03\x04", 8),
      ("key", SEND_KEY, b"\x67\x12", 8),
      ("read_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE[:6], 19),
      ("read_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE[:6], 21),
      ("final_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE[:6], 19),
      ("final_vin", READ_VIN, SYNTHETIC_VIN_RESPONSE[:6], 21),
    )
    for stage, request, prefix, declared_length in cases:
      with self.subTest(stage=stage, declared_length=declared_length):
        self._prepare_stage(stage)
        self.assertTrue(self._diag_tx(request))
        self._diag_rx(self._first_frame(prefix, declared_length))
        self.assertFalse(self._diag_tx(FLOW_CONTROL))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_multiframe_sequence_and_completion_are_required(self):
    self._prepare_stage("read_vin")
    self.assertTrue(self._diag_tx(READ_VIN))
    response = SYNTHETIC_VIN_RESPONSE
    first_frame = bytes([0x10, len(response)]) + response[:6]
    self._diag_rx(first_frame)
    self.assertTrue(self._diag_tx(FLOW_CONTROL))
    self.assertFalse(self._diag_tx(REQUEST_SEED))
    self._diag_rx(b"\x22" + response[6:13])
    self.assertFalse(self._diag_tx(REQUEST_SEED))
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_response_address_bus_length_padding_and_pci_are_strict(self):
    self._prepare_stage("default")
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))
    self._diag_rx(b"\x02\x50\x01\x00\x00\x00\x00\x00", addr=RADAR_DIAG_RX_ADDR - 1)
    self._single_response(b"\x50\x01")
    self.assertTrue(self._diag_tx(EXTENDED_SESSION))

    malformed = (
      (b"\x02\x50\x01\x00\x00\x00\x00\x00", RADAR_DIAG_BUS + 1),
      (b"\x02\x50\x01\x00\x00\x00\x00", RADAR_DIAG_BUS),
      (b"\x02\x50\x01\x01\x00\x00\x00\x00", RADAR_DIAG_BUS),
      (b"\x30\x00\x00\x00\x00\x00\x00\x00", RADAR_DIAG_BUS),
    )
    for response, bus in malformed:
      with self.subTest(response=response, bus=bus):
        self._prepare_stage("default")
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._diag_rx(response, bus=bus)
        self.assertFalse(self._diag_tx(EXTENDED_SESSION))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_drive_state_and_stalk_cannot_cancel_or_enable_diagnostics(self):
    self._prepare_stage("default")
    for msg in (self._gear_msg(4), self._gear_msg(0), self._speed_msg(30),
                self._di_brake_msg(True), self._di_brake_msg(False),
                self._user_brake_msg(True), self._user_brake_msg(False),
                self._pcm_status_msg(True)):
      self._rx(msg)
      self.assertFalse(self.safety.get_controls_allowed())

    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_gtw_emulation_survives_diagnostic_interleaving(self):
    flags = TeslaPreAPSafetyFlags.RADAR_VIN_LEARN | TeslaPreAPSafetyFlags.RADAR_BEHIND_NOSECONE
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(flags))
    self.safety.init_tests()
    self.assertEqual(self._gtw_emissions(), [])

    self.assertTrue(self._diag_tx(TESTER_PRESENT))
    vip_data = b"\x12\x34\x56\x78\x9a\xbc\xde\xf0"
    config_data = b"\xff" * 8
    self._rx(libsafety_py.make_CANPacket(0x405, 0, vip_data))
    self._rx(libsafety_py.make_CANPacket(0x398, 0, config_data))

    expected_config = b"\x7f\xf7\xff\xff\x1f\x0f\xff\x4c"
    self.assertEqual(self._gtw_emissions(), [
      (0x2B9, 1, vip_data),
      (0x2A9, 1, expected_config),
    ])

    self._single_response(b"\x7e\x00")
    self.assertEqual(len(self._gtw_emissions()), 2)
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_gtw_debug_sink_resets_with_test_and_safety_initialization(self):
    vip_frame = libsafety_py.make_CANPacket(0x405, 0, b"\x12\x34\x56\x78\x9a\xbc\xde\xf0")
    self._rx(vip_frame)
    self.assertEqual(len(self._gtw_emissions()), 1)

    self.safety.init_tests()
    self.assertEqual(self._gtw_emissions(), [])

    self._rx(vip_frame)
    self.assertEqual(len(self._gtw_emissions()), 1)
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(TeslaPreAPSafetyFlags.RADAR_VIN_LEARN))
    self.assertEqual(self._gtw_emissions(), [])

  def test_release_frames_only_in_every_latched_state(self):
    for state in ("active", "poisoned", "cleanup_pending"):
      with self.subTest(state=state):
        self._enter_latched_state(state)
        self._assert_non_pedal_release_policy()

  def test_poison_conditions_remain_fail_closed_until_matching_cleanup(self):
    poisoners = ("out_of_order", "unexpected_response", "controls", "timeout")
    for poisoner in poisoners:
      with self.subTest(poisoner=poisoner):
        self._setup_safety_hooks()
        self.assertTrue(self._diag_tx(TESTER_PRESENT))
        if poisoner == "out_of_order":
          self.assertFalse(self._diag_tx(REQUEST_SEED))
        elif poisoner == "unexpected_response":
          self._single_response(b"\x67\x11\x01\x02\x03\x04")
        elif poisoner == "controls":
          self.safety.set_controls_allowed(True)
          self.assertTrue(self._tx(self._epas_control_msg(0)))
          self.assertFalse(self.safety.get_controls_allowed())
        else:
          self.safety.set_timer(30000000)
          self.assertFalse(self._diag_tx(EXTENDED_SESSION))

        self.assertFalse(self._diag_tx(START_ROUTINE))
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(self._angle_cmd_msg(0, 1)))
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._single_response(b"\x50\x03")
        self.assertFalse(self._diag_tx(TESTER_PRESENT))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._single_response(b"\x50\x01")

        self.safety.set_controls_allowed(True)
        self._rx(self._angle_meas_msg(0))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))

  def test_inactivity_timeout_boundary_is_exact_and_wrap_safe(self):
    self._setup_safety_hooks()
    self.assertTrue(self._diag_tx(TESTER_PRESENT))
    self._single_response(b"\x7e\x00")
    self.safety.set_timer(29999999)
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

    self._setup_safety_hooks()
    self.safety.set_timer(0xFFFFFF00)
    self.assertTrue(self._diag_tx(TESTER_PRESENT))
    self.safety.set_timer((0xFFFFFF00 + 30000000) & 0xFFFFFFFF)
    self.assertFalse(self._diag_tx(EXTENDED_SESSION))
    self.assertTrue(self._diag_tx(DEFAULT_SESSION))

  def test_cleanup_is_available_from_each_protocol_stage(self):
    for stage, _, _ in self.REQUEST_STAGES:
      if stage == "default":
        continue
      with self.subTest(stage=stage):
        self._prepare_stage(stage)
        if stage == "tester":
          self.assertTrue(self._diag_tx(TESTER_PRESENT))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._single_response(b"\x50\x01")
        self.safety.set_controls_allowed(True)
        self._rx(self._angle_meas_msg(0))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))

  def test_reinitialization_exposes_cleanup_only_bootstrap_from_each_stage(self):
    for stage, _, _ in self.REQUEST_STAGES:
      with self.subTest(stage=stage):
        self._prepare_stage(stage)
        self._setup_safety_hooks()
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self.assertFalse(self._diag_tx(TESTER_PRESENT))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._single_response(b"\x50\x01")
        self.safety.set_controls_allowed(True)
        self._rx(self._angle_meas_msg(0))
        self.assertTrue(self._tx(self._angle_cmd_msg(0, 1)))

  def test_cleanup_only_bootstrap_rejects_every_other_diagnostic_request(self):
    for request in (TESTER_PRESENT, EXTENDED_SESSION, REQUEST_SEED, SEND_KEY,
                    START_ROUTINE, STOP_ROUTINE, REQUEST_RESULTS, READ_VIN, FLOW_CONTROL):
      with self.subTest(request=request):
        self._setup_safety_hooks()
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self.assertFalse(self._diag_tx(request))
        self.assertTrue(self._diag_tx(DEFAULT_SESSION))
        self._single_response(b"\x50\x01")


class TestTeslaPreAPRadarVin(TeslaPreAPRadarVinTestMixin, unittest.TestCase):
  __test__ = True

  def setUp(self):
    super().setUp()
    self._setup_safety_hooks()


class TestTeslaPreAPRadarVinWithPedal(TestTeslaPreAPWithPedal):
  __test__ = True

  def _setup_safety_hooks(self):
    flags = TeslaPreAPSafetyFlags.ENABLE_PEDAL | TeslaPreAPSafetyFlags.RADAR_VIN_LEARN
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaPreap, int(flags))
    self.safety.init_tests()

  def _diag_tx(self, data):
    return self._tx(libsafety_py.make_CANPacket(RADAR_DIAG_TX_ADDR, RADAR_DIAG_BUS, data))

  def test_diagnostic_latch_allows_only_released_pedal_command(self):
    for state in ("active", "poisoned", "cleanup_pending"):
      with self.subTest(state=state):
        self._setup_safety_hooks()
        if state == "active":
          self.assertTrue(self._diag_tx(TESTER_PRESENT))
        elif state == "poisoned":
          self.assertTrue(self._diag_tx(TESTER_PRESENT))
          self.assertFalse(self._diag_tx(REQUEST_SEED))
        else:
          self.assertTrue(self._diag_tx(DEFAULT_SESSION))

        released = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 0})
        high_release = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 100, "ENABLE": 0})
        enabled = self.packer.make_can_msg_safety("GAS_COMMAND", 0, {"GAS_COMMAND": 0, "ENABLE": 1})
        self.assertTrue(self._tx(released))
        self.assertFalse(self._tx(high_release))
        self.safety.set_controls_allowed(True)
        self.assertFalse(self._tx(enabled))
        self.assertFalse(self.safety.get_controls_allowed())


if __name__ == "__main__":
  unittest.main()
