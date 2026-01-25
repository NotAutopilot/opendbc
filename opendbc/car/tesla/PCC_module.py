"""
NAP Pedal Cruise Controller (PCC) Module

Controls the pedal interceptor for longitudinal control on pre-AP Tesla Model S.
Handles cruise engagement via double-pull of the cruise stalk.

Migrated from Tinkla with NAP-specific parameter names and event types.
"""
import time
import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons, CarControllerParams
from opendbc.car.tesla.nap_params import load_int_param, NAPParamKeys
from opendbc.car.tesla.tunes import (
    PEDAL_BP, PEDAL_V, PEDAL_DI_PRESSED, PEDAL_DI_MIN,
    PEDAL_CALIBRATED, transform_di_to_pedal
)
from cereal import car

ACCEL_MAX = 2.5  # m/s^2
ACCEL_MIN = CarControllerParams.PREAP_ACCEL_MIN
MIN_SAFE_DIST_M = 3.

_DT = 0.01  # 100Hz

# Max distances
MAX_RADAR_DISTANCE = 120.0  # max distance to consider radar reading
MAX_PEDAL_VALUE_AVG = 100
MAX_BRAKE_VALUE = 1.  # iBooster fully pressed

PEDAL_HYST_GAP = 1.0  # don't change pedal command for small oscillations

# Torque thresholds
TORQUE_LEVEL_ACC = 0.0
TORQUE_LEVEL_DECEL = -30.0

MIN_PCC_V_KPH = 0.0
MAX_PCC_V_KPH = 270.0

# Pull the cruise stalk twice in this many ms for a 'double pull'
STALK_DOUBLE_PULL_MS = 750
# Do not show max regen error 2 seconds after engagement
TIMEOUT_REGEN_ERROR = 2000

# Load pedal profile from NAP params (1-4 index, subtract 1 for 0-based)
PEDAL_PROFILE = int(load_int_param(NAPParamKeys.PEDAL_PROFILE, 1)) - 1
if PEDAL_PROFILE < 0 or PEDAL_PROFILE >= len(PEDAL_V):
    PEDAL_PROFILE = 0


class PCCState:
    """Possible states of the PCC system, following DI_cruiseState naming scheme"""
    OFF = 0  # Disabled by UI
    STANDBY = 1  # Ready to be engaged
    ENABLED = 2  # Engaged
    NOT_READY = 9  # Not ready due to car state


def _current_time_millis():
    return int(round(time.monotonic() * 1000))


class PCCController:
    """
    Pedal Cruise Controller for pre-AP Tesla Model S.

    Handles:
    - Stalk double-pull engagement
    - Pedal value calculation based on acceleration target
    - Speed limit integration
    - iBooster brake coordination
    """

    def __init__(self, long_controller, tesla_can, pedalcan, CP):
        self.CP = CP
        self.LongCtr = long_controller
        self.tesla_can = tesla_can
        self.pedalcan = pedalcan

        # State tracking
        self.human_cruise_action_time = 0
        self.pcc_available = self.prev_pcc_available = False
        self.pedal_timeout_frame = 0
        self.accelerator_pedal_pressed = self.prev_accelerator_pedal_pressed = False
        self.automated_cruise_action_time = 0
        self.last_angle = 0.0
        self.lead_1 = None
        self.last_update_time = 0
        self.enable_pedal_cruise = False

        # Stalk tracking for double-pull detection
        self.stalk_pull_time_ms = 0
        self.prev_stalk_pull_time_ms = -1000
        self.prev_cruise_state = 0
        self.prev_cruise_buttons = CruiseButtons.IDLE

        # Speed control
        self.pedal_speed_kph = 0.0
        self.speed_limit_kph = 0.0
        self.prev_speed_limit_kph = 0.0

        # Pedal control state
        self.pedal_idx = 0
        self.pedal_steady = 0.0
        self.prev_tesla_accel = 0.0
        self.prev_tesla_pedal = 0.0
        self.prev_tesla_brake = 0.0
        self.torqueLevel_last = 0.0
        self.prev_v_ego = 0.0

        # Zero torque calibration
        self.PedalForZeroTorque = 0.0
        self.lastTorqueForPedalForZeroTorque = TORQUE_LEVEL_DECEL
        self.lastApidForPedalForZeroTorque = 0.
        self.prev_a_pid = 0.
        self.last_max_regen_time_ms = -1000.

        # PID values
        self.v_pid = 0.0
        self.a_pid = 0.0
        self.last_output_gb = 0.0
        self.last_speed_kph = None

        # Speed smoothing
        self.v_acc_start = 0.0
        self.a_acc_start = 0.0
        self.v_acc = 0.0
        self.v_acc_sol = 0.0
        self.v_acc_future = 0.0
        self.a_acc = 0.0
        self.a_acc_sol = 0.0
        self.v_cruise = 0.0
        self.a_cruise = 0.0

        # Radar tracking
        self.lead_last_seen_time_ms = 0
        self.continuous_lead_sightings = 0

    def update_stat(self, CS, frame):
        """
        Update PCC state based on car state and user inputs.

        Returns list of CAN messages (typically empty or pedal reset command).
        """
        if not self.LongCtr.CP.openpilotLongitudinalControl:
            self.pcc_available = False
            return []

        if not CS.enablePedal:
            self.pcc_available = False
            return []

        self._update_pedal_state(CS, frame)

        can_sends = []
        if not self.pcc_available:
            timed_out = frame >= self.pedal_timeout_frame
            if timed_out or CS.pedal_interceptor_state > 0:
                if frame % 50 == 0:
                    # Send reset command
                    idx = self.pedal_idx
                    self.pedal_idx = (self.pedal_idx + 1) % 16
                    can_sends.append(
                        self.tesla_can.create_pedal_command_msg(0, 0, idx, self.pedalcan)
                    )
            return can_sends

        # Disable on brake
        if CS.realBrakePressed and self.enable_pedal_cruise:
            CS.longCtrlEvent = car.CarEvent.EventName.napPedalCruiseDisabled
            self.enable_pedal_cruise = False

        # Process stalk movement
        curr_time_ms = _current_time_millis()
        speed_uom_kph = 1.0
        if CS.speed_units == "MPH":
            speed_uom_kph = CV.MPH_TO_KPH

        if (
            CS.cruise_buttons == CruiseButtons.MAIN
            and self.prev_cruise_buttons != CruiseButtons.MAIN
        ):
            self.prev_stalk_pull_time_ms = self.stalk_pull_time_ms
            self.stalk_pull_time_ms = curr_time_ms
            double_pull = (
                self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms
                < STALK_DOUBLE_PULL_MS
            )
            ready = CS.enablePedal

            if ready and double_pull:
                # A double pull enables PCC
                if not self.enable_pedal_cruise and PEDAL_CALIBRATED:
                    CS.longCtrlEvent = car.CarEvent.EventName.napPedalCruiseEnabled
                if not PEDAL_CALIBRATED:
                    CS.longCtrlEvent = car.CarEvent.EventName.napPedalCalibrationNeeded
                else:
                    self.enable_pedal_cruise = True
                    # Set PCC speed to current speed, rounded to user's units
                    current_speed_kph_uom_rounded = (
                        int(CS.out.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
                    )
                    self.pedal_speed_kph = max(
                        current_speed_kph_uom_rounded, self.speed_limit_kph
                    )

        # Handle pressing the cancel button
        elif CS.cruise_buttons == CruiseButtons.CANCEL:
            if self.enable_pedal_cruise:
                CS.longCtrlEvent = car.CarEvent.EventName.napPedalCruiseDisabled
            self.enable_pedal_cruise = False
            self.pedal_speed_kph = 0.0
            self.stalk_pull_time_ms = 0
            self.prev_stalk_pull_time_ms = -1000

        # Handle pressing up and down buttons
        elif self.enable_pedal_cruise and CS.cruise_buttons != self.prev_cruise_buttons:
            actual_speed_kph_uom_rounded = (
                int(CS.out.vEgo * CV.MS_TO_KPH / speed_uom_kph + 0.5) * speed_uom_kph
            )
            if CS.cruise_buttons == CruiseButtons.RES_ACCEL:
                self.pedal_speed_kph = (
                    max(self.pedal_speed_kph, actual_speed_kph_uom_rounded)
                    + speed_uom_kph
                )
            elif CS.cruise_buttons == CruiseButtons.RES_ACCEL_2ND:
                self.pedal_speed_kph = (
                    max(self.pedal_speed_kph, actual_speed_kph_uom_rounded)
                    + 5 * speed_uom_kph
                )
            elif CS.cruise_buttons == CruiseButtons.DECEL_SET:
                self.pedal_speed_kph = self.pedal_speed_kph - speed_uom_kph
            elif CS.cruise_buttons == CruiseButtons.DECEL_2ND:
                self.pedal_speed_kph = self.pedal_speed_kph - 5 * speed_uom_kph

            # Clip PCC speed
            self.pedal_speed_kph = np.clip(
                self.pedal_speed_kph, MIN_PCC_V_KPH, MAX_PCC_V_KPH
            )
            if not PEDAL_CALIBRATED:
                CS.longCtrlEvent = car.CarEvent.EventName.napPedalCalibrationNeeded

        # If cruise control disabled externally, disable PCC too
        elif self.enable_pedal_cruise and CS.cruise_state and not CS.enablePedal:
            self.enable_pedal_cruise = False
            CS.longCtrlEvent = car.CarEvent.EventName.napPedalCruiseDisabled

        # A single pull disables PCC (falling back to just steering)
        # Wait some time in case a double pull comes
        elif (
            self.enable_pedal_cruise
            and curr_time_ms - self.stalk_pull_time_ms > STALK_DOUBLE_PULL_MS
            and self.stalk_pull_time_ms - self.prev_stalk_pull_time_ms > STALK_DOUBLE_PULL_MS
        ):
            self.enable_pedal_cruise = False
            CS.longCtrlEvent = car.CarEvent.EventName.napPedalCruiseDisabled

        # Update prev state
        self.prev_cruise_buttons = CS.cruise_buttons
        self.prev_cruise_state = CS.cruise_state

        return can_sends

    def update_pdl(
        self,
        enabled,
        CS,
        frame,
        actuators,
        v_target,
        a_pid,
        a_target,
        pcm_override,
        speed_limit_ms,
        set_speed_limit_active,
        speed_limit_offset,
        alca_enabled,
        radSt
    ):
        """
        Calculate pedal value based on acceleration target.

        Returns (pedal_value, brake_value, enable_pedal, pedal_idx)
        """
        if not self.LongCtr.CP.openpilotLongitudinalControl:
            return 0.0, 0.0, -1, -1

        if not CS.enablePedal:
            return 0.0, 0.0, -1, -1

        if radSt is not None:
            self.lead_1 = radSt.radarState.leadOne

        self.prev_speed_limit_kph = self.speed_limit_kph

        # Determine pedal "zero" - position for cruising with no torque
        if (
            CS.torqueLevel < TORQUE_LEVEL_ACC
            and CS.torqueLevel > TORQUE_LEVEL_DECEL
            and CS.out.vEgo >= 10.0 * CV.MPH_TO_MS
            and abs(CS.torqueLevel) < abs(self.lastTorqueForPedalForZeroTorque)
        ):
            self.PedalForZeroTorque = self.prev_tesla_pedal
            self.lastTorqueForPedalForZeroTorque = CS.torqueLevel
            self.lastApidForPedalForZeroTorque = self.prev_a_pid
        self.prev_a_pid = a_pid

        # Update speed limit
        if set_speed_limit_active and speed_limit_ms > 0:
            self.speed_limit_kph = (speed_limit_ms + speed_limit_offset) * CV.MS_TO_KPH
            if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
                self.pedal_speed_kph = self.speed_limit_kph
        else:
            self.speed_limit_kph = 0.0

        if not self.pcc_available or not enabled or not self.enable_pedal_cruise:
            return 0.0, 0.0, 0, self.pedal_idx

        if CS.out.gasPressed:
            return 0.0, 0.0, 0, self.pedal_idx

        # Calculate pedal value using longitudinal MPC output
        ZERO_ACCEL = self.PedalForZeroTorque

        # Regen braking deceleration varies with speed
        REGEN_DECEL_BP = [10., 20.]
        REGEN_DECEL_V = [-0.8, -1.45]
        REGEN_DECEL = np.interp(CS.out.vEgo, REGEN_DECEL_BP, REGEN_DECEL_V)

        MAX_PEDAL_BP = PEDAL_BP
        MAX_PEDAL_V = PEDAL_V[PEDAL_PROFILE]
        MAX_PEDAL_VALUE = np.interp(CS.out.vEgo, MAX_PEDAL_BP, MAX_PEDAL_V)

        MIN_PEDAL_REGEN_VALUE = PEDAL_DI_MIN

        if CS.out.vEgo < 5 * CV.MPH_TO_MS:
            ZERO_ACCEL = 0.

        ACCEL_LOOKUP_BP = [REGEN_DECEL, 0, ACCEL_MAX]
        ACCEL_LOOKUP_V = [MIN_PEDAL_REGEN_VALUE, ZERO_ACCEL, MAX_PEDAL_VALUE]

        # Cap pedal rate of change for smooth acceleration
        PEDAL_MAX_DOWN = MAX_PEDAL_VALUE * _DT / 0.4
        PEDAL_MAX_UP = (MAX_PEDAL_VALUE - self.prev_tesla_pedal) * _DT

        BRAKE_LOOKUP_BP = [-4.5, 0.]
        BRAKE_LOOKUP_V = [1.0, 0.]

        enable_pedal = 1.0 if self.enable_pedal_cruise else 0.0
        tesla_pedal = int(round(np.interp(a_pid, ACCEL_LOOKUP_BP, ACCEL_LOOKUP_V)))

        # Pedal hysteresis when close to set speed
        if abs(CS.out.vEgo * CV.MS_TO_KPH - self.pedal_speed_kph) < 0.8 and CS.out.vEgo > 5.:
            tesla_pedal = self.pedal_hysteresis(tesla_pedal, enable_pedal)

        if (CS.out.vEgo < 0.1) and (a_target < 0.01):
            # Hold brake at standstill
            tesla_brake = 0.43
        else:
            tesla_brake = np.clip(np.interp(a_pid, BRAKE_LOOKUP_BP, BRAKE_LOOKUP_V), 0, 1)

        # If gas pedal pressed, don't apply brake
        if CS.pedal_interceptor_value > (PEDAL_DI_PRESSED + 5.):
            tesla_brake = 0

        if CS.has_ibooster_ecu and CS.brakeUnavailable:
            CS.longCtrlEvent = car.CarEvent.EventName.napIBoosterFault

        tesla_pedal = np.clip(tesla_pedal, self.prev_tesla_pedal - PEDAL_MAX_DOWN, self.prev_tesla_pedal + PEDAL_MAX_UP)
        tesla_pedal = np.clip(tesla_pedal, MIN_PEDAL_REGEN_VALUE, MAX_PEDAL_VALUE)

        if CS.ibstBrakeApplied:
            # Wait for iBooster to release before accelerating
            tesla_pedal = min(tesla_pedal, MIN_PEDAL_REGEN_VALUE)

        # Show max regen warning if no iBooster
        if (
            (not CS.has_ibooster_ecu)
            and tesla_pedal <= 0.95 * MIN_PEDAL_REGEN_VALUE
            and enable_pedal == 1
            and (_current_time_millis() - self.stalk_pull_time_ms) > TIMEOUT_REGEN_ERROR
        ):
            CS.pccEvent = car.CarEvent.EventName.napMaxRegenActive
        else:
            CS.pccEvent = None

        if enable_pedal == 1 and not PEDAL_CALIBRATED:
            CS.pccEvent = car.CarEvent.EventName.napPedalCalibrationNeeded

        self.prev_tesla_brake = tesla_brake * enable_pedal
        self.torqueLevel_last = CS.torqueLevel
        self.prev_tesla_pedal = tesla_pedal * enable_pedal
        self.prev_v_ego = CS.out.vEgo

        pedal2send = self.prev_tesla_pedal
        if enable_pedal == 1:
            pedal2send = transform_di_to_pedal(pedal2send)

        return pedal2send, self.prev_tesla_brake, enable_pedal, self.pedal_idx

    def pedal_hysteresis(self, pedal, enabled):
        """Apply hysteresis to prevent small oscillations in pedal command"""
        if not enabled:
            self.pedal_steady = 0.0
        elif pedal > self.pedal_steady + PEDAL_HYST_GAP:
            self.pedal_steady = pedal - PEDAL_HYST_GAP
        elif pedal < self.pedal_steady - PEDAL_HYST_GAP:
            self.pedal_steady = pedal + PEDAL_HYST_GAP
        return self.pedal_steady

    def _update_pedal_state(self, CS, frame):
        """Update pedal availability state based on CAN timeout and interceptor state"""
        if CS.pedal_idx != CS.prev_pedal_idx:
            # Timeout pedal after 500ms without receiving a new CAN message
            self.pedal_timeout_frame = frame + 50

        self.prev_pcc_available = self.pcc_available
        pedal_ready = (
            frame < self.pedal_timeout_frame and CS.pedal_interceptor_state == 0
        )
        # Mark pedal unavailable while traditional cruise is on
        self.pcc_available = (pedal_ready and CS.enablePedal) or CS.autopilot_disabled
