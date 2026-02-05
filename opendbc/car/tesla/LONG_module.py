"""
NAP Longitudinal Controller Module

Coordinates PCC and ACC modules for pre-AP Tesla Model S longitudinal control.
Handles speed limits, iBooster braking, and pedal command generation.

Migrated from Tinkla with NAP-specific parameter names and event types.
"""
import numpy as np
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import (
    CarControllerParams, CruiseState, CruiseButtons,
    TESLA_MAX_ACCEL, TESLA_MIN_ACCEL, CAR, CAN_CHASSIS, CAN_POWERTRAIN, PREAP_CARS
)
from opendbc.car.tesla.ACC_module import ACCController
from opendbc.car.tesla.PCC_module import PCCController
from opendbc.car.tesla.nap_params import load_bool_param, load_float_param, NAPParamKeys
from opendbc.car.tesla.speed_utils.fleet_speed import FleetSpeed


# Acceleration multipliers for smooth response
ACCEL_MULTIPLIERS_BP = [0.0, 5.0, 10.0, 30.0]
ACCEL_MULT_SPEED_V = [1.5, 1.3, 1.2, 1.0]
ACCEL_MULT_SPEED_DELTA_V = [1.0, 1.01, 1.05, 1.1]
ACCEL_MULT_ACCEL_PERC_V = [1.0, 1.0, 1.05, 1.1]

FLEET_SPEED_ACCEL = -0.5  # m/s^2 how fast to reduce speed to match fleet

# Brake factor varies with speed
BRAKE_FACTOR_BP = [18., 28.]
BRAKE_FACTOR_V = [1.15, 1.45]

# Load brake factor from NAP params
BRAKE_FACTOR = load_float_param(NAPParamKeys.BRAKE_FACTOR, 1.0)


def _is_present(lead) -> bool:
    """Check if lead vehicle is present"""
    return bool((lead is not None) and (lead.dRel > 0))


def _get_accel_multiplier(speed: float, speed_target: float, accel: float) -> float:
    """Calculate acceleration multiplier for smooth response"""
    mult = 1.0
    # Only for positive acceleration
    if accel <= 0:
        return mult
    mult = mult * np.interp(speed, ACCEL_MULTIPLIERS_BP, ACCEL_MULT_SPEED_V)
    mult = mult * np.interp(abs(speed - speed_target), ACCEL_MULTIPLIERS_BP, ACCEL_MULT_SPEED_DELTA_V)
    return mult


class LONGController:
    """
    Longitudinal Controller for Tesla.

    Coordinates PCC (Pedal Cruise Control) and ACC (Adaptive Cruise Control)
    modules for pre-AP and HW1+ vehicles with openpilot longitudinal control.

    Responsibilities:
    - Initialize PCC/ACC controllers for pre-AP vehicles
    - Process speed limit information
    - Generate pedal and iBooster commands
    - Handle AP1/AP2 longitudinal control messages
    """

    def __init__(self, CP, packer, tesla_can, pedalcan):
        self.CP = CP
        self.packer = packer
        self.tesla_can = tesla_can
        self.pedalcan = pedalcan

        # Longitudinal state
        self.v_target = None
        self.a_target = 0.
        self.j_target = 0.
        self.lead_1 = None
        self.long_control_counter = 1

        # iBooster state
        self.has_ibooster_ecu = False
        self.ibooster_idx = 0
        self.apply_brake = 0.0

        # Load speed limit params from NAP
        self.speed_limit_offset_uom = load_float_param(NAPParamKeys.SPEED_LIMIT_OFFSET, 0.0)
        self.speed_limit_offset_ms = 0.0
        self.adjustSpeedWithSpeedLimit = load_bool_param(NAPParamKeys.ADJUST_ACC_WITH_SPEED_LIMIT, True)
        self.adjustSpeedRelative = load_bool_param(NAPParamKeys.SPEED_LIMIT_USE_RELATIVE, False)
        self.useLongControlData = load_bool_param(NAPParamKeys.USE_LONG_CONTROL_DATA, False)
        self.autopilot_disabled = load_bool_param(NAPParamKeys.AUTOPILOT_DISABLED, False)

        # Fleet speed averaging
        average_speed_over_x_suggestions = 5  # 1 second @ 5Hz
        self.speed_limit_uom = 0.
        self.prev_speed_limit_uom = 0.
        self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)
        self.fleet_speed_ms = 0.

        # State tracking
        self.prev_enabled = False
        self.speed_limit_ms = 0.
        self.prev_speed_limit_ms = 0.
        self.prev_speed_limit_ms_das = 0.
        self.ap1_adjusting_speed = False
        self.ap1_speed_target = 0.
        self.set_speed_limit_active = False
        self.longPlan = None

        # Initialize PCC/ACC for pre-AP or when autopilot is disabled
        self.ACC = None
        self.PCC = None
        if CP.carFingerprint in PREAP_CARS or self.autopilot_disabled:
            self.ACC = ACCController(self)
            self.PCC = PCCController(self, tesla_can, pedalcan, CP)

    def update(self, enabled, CS, frame, actuators, cruise_cancel, pcm_speed, pcm_override, long_plan, radar_state):
        """
        Main update function called each frame.

        Handles:
        - Speed limit processing
        - PCC/ACC coordination
        - Pedal and iBooster command generation
        - AP1/AP2 longitudinal control

        Returns list of CAN messages to send.
        """
        messages = []
        self.has_ibooster_ecu = CS.has_ibooster_ecu

        # Apply brake factor to negative accel
        my_accel = actuators.accel
        if my_accel < 0:
            my_accel = my_accel * BRAKE_FACTOR

        # Clip accel to limits
        target_accel = np.clip(my_accel, TESLA_MIN_ACCEL, TESLA_MAX_ACCEL)

        # Pre-AP has additional speed-dependent brake factor
        if self.CP.carFingerprint in PREAP_CARS or self.autopilot_disabled:
            target_accel = np.clip(
                my_accel * np.interp(CS.out.vEgo, BRAKE_FACTOR_BP, BRAKE_FACTOR_V),
                TESLA_MIN_ACCEL, TESLA_MAX_ACCEL
            )

        max_accel = 0 if target_accel < 0 else target_accel
        min_accel = 0 if target_accel > 0 else target_accel

        max_jerk = CarControllerParams.JERK_LIMIT_MAX
        min_jerk = CarControllerParams.JERK_LIMIT_MIN

        tesla_jerk_limits = [min_jerk, max_jerk]
        tesla_accel_limits = [min_accel, max_accel]

        # Reset fleet speed averager on enable
        if enabled and not self.prev_enabled:
            self.fleet_speed.reset_averager()

        # Update speed limit at 10Hz
        if frame % 10 == 0:
            self.speed_limit_ms = CS.speed_limit_ms
            if self.speed_limit_ms != self.prev_speed_limit_ms:
                self.fleet_speed.reset_averager()
                self.prev_speed_limit_ms = self.speed_limit_ms
            self.set_speed_limit_active = self.adjustSpeedWithSpeedLimit if self.speed_limit_ms > 0 else False

        # Update speed limit offset at 1Hz
        if frame % 100 == 0:
            if self.CP.carFingerprint not in PREAP_CARS:
                self.speed_limit_offset_uom = CS.userSpeedLimitOffsetMS
            if self.adjustSpeedRelative:
                if self.speed_limit_ms > 0:
                    self.speed_limit_offset_ms = self.speed_limit_offset_uom * self.speed_limit_ms / 100.0
            else:
                if CS.speed_units == "KPH":
                    self.speed_limit_offset_ms = self.speed_limit_offset_uom * CV.KPH_TO_MS
                elif CS.speed_units == "MPH":
                    self.speed_limit_offset_ms = self.speed_limit_offset_uom * CV.MPH_TO_MS

        # Exit if not openpilot long control
        if not self.CP.openpilotLongitudinalControl:
            return messages

        # Pre-AP Model S handling
        if self.CP.carFingerprint in PREAP_CARS or self.autopilot_disabled:
            messages = self._update_preap(
                enabled, CS, frame, actuators, target_accel, pcm_speed, pcm_override,
                long_plan, radar_state, tesla_accel_limits, tesla_jerk_limits
            )

        self.prev_enabled = enabled
        return messages

    def _update_preap(self, enabled, CS, frame, actuators, target_accel, pcm_speed,
                      pcm_override, long_plan, radar_state, tesla_accel_limits, tesla_jerk_limits):
        """Handle pre-AP Model S longitudinal control"""
        messages = []

        # Update PCC module info
        pedal_can_sends = self.PCC.update_stat(CS, frame)
        if len(pedal_can_sends) > 0:
            messages.extend(pedal_can_sends)

        # When "Disable Cruise Control" toggle is enabled (enablePedalOverCC=True) and PCC is active,
        # actively cancel stock CC if it tries to engage. This prevents the two systems from fighting.
        if (CS.enablePedalOverCC and
            self.PCC.pcc_available and
            self.PCC.enable_pedal_cruise and
            CruiseState.is_enabled_or_standby(CS.cruise_state) and
            frame % 10 == 0):  # Send at 10Hz to ensure it takes effect
            stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
            messages.insert(0, self.tesla_can.create_action_request(
                msg_stw_actn_req=CS.msg_stw_actn_req,
                button_to_press=CruiseButtons.CANCEL,
                bus=CAN_CHASSIS[self.CP.carFingerprint],
                counter=stlk_counter
            ))

        if self.PCC.pcc_available:
            self.ACC.enable_adaptive_cruise = False
        else:
            # Update ACC module info when PCC not available
            self.PCC.enable_pedal_cruise = False
            self.ACC.update_stat(CS, True)

        # Update CS.v_cruise_pcm based on module selected
        speed_uom_kph = 1.0

        # Cruise state: 0=unavailable, 1=available, 2=enabled, 3=hold
        if CS.carNotInDrive:
            CS.cc_state = 0
        else:
            CS.cc_state = 1

        CS.speed_control_enabled = 0

        if enabled:
            if CS.speed_units == "MPH":
                speed_uom_kph = CV.KPH_TO_MPH

            if self.ACC.enable_adaptive_cruise:
                CS.acc_speed_kph = self.ACC.acc_speed_kph
            elif self.PCC.enable_pedal_cruise:
                CS.acc_speed_kph = self.PCC.pedal_speed_kph
            else:
                CS.acc_speed_kph = max(0.0, CS.out.vEgo * CV.MS_TO_KPH)

            CS.v_cruise_pcm = CS.acc_speed_kph * speed_uom_kph

            if (self.PCC.pcc_available and self.PCC.enable_pedal_cruise) or self.ACC.enable_adaptive_cruise:
                CS.speed_control_enabled = 1
                CS.cc_state = 2
                if not self.ACC.adaptive:
                    CS.cc_state = 3  # HOLD for non-adaptive
        else:
            if CS.cruise_state == CruiseState.OVERRIDE:
                CS.cc_state = 3

        CS.adaptive_cruise = (
            1
            if (not self.PCC.pcc_available and self.ACC.adaptive) or self.PCC.pcc_available
            else 0
        )
        CS.adaptive_cruise_enabled = (
            (self.ACC.adaptive and self.ACC.enable_adaptive_cruise and not self.PCC.pcc_available)
            or self.PCC.enable_pedal_cruise
        )
        CS.pcc_available = self.PCC.pcc_available
        CS.pcc_enabled = self.PCC.enable_pedal_cruise

        # Get longitudinal plan targets
        if long_plan and long_plan.longitudinalPlan:
            self.longPlan = long_plan.longitudinalPlan
            self.v_target = self.longPlan.speeds[-1]

        # ACC button presses at 5Hz (when PCC not available)
        if not self.PCC.pcc_available and frame % 20 == 0:
            cruise_btn = self.ACC.update_acc(
                enabled,
                CS,
                frame,
                actuators,
                self.v_target,
                self.speed_limit_ms * CV.MS_TO_KPH,
                self.set_speed_limit_active,
                self.speed_limit_offset_ms * CV.MS_TO_KPH,
            )
            if cruise_btn:
                # Insert message first since it races against real stalk messages
                stlk_counter = ((CS.msg_stw_actn_req['MC_STW_ACTN_RQ'] + 1) % 16)
                messages.insert(0, self.tesla_can.create_action_request(
                    msg_stw_actn_req=CS.msg_stw_actn_req,
                    button_to_press=cruise_btn,
                    bus=CAN_CHASSIS[self.CP.carFingerprint],
                    counter=stlk_counter
                ))

        # Pedal commands at 100Hz (when PCC available)
        apply_accel = 0.0
        if self.PCC.pcc_available and frame % 1 == 0:
            if self.longPlan:
                self.v_target = self.longPlan.speeds[0]
                self.a_target = self.longPlan.accels[0]

            if self.v_target is None:
                self.v_target = CS.out.vEgo
            target_speed = max(self.v_target, 0)

            apply_accel, self.apply_brake, accel_needed, accel_idx = self.PCC.update_pdl(
                enabled,
                CS,
                frame,
                actuators,
                target_speed,
                target_accel,
                self.a_target,
                pcm_override,
                self.speed_limit_ms,
                self.set_speed_limit_active,
                self.speed_limit_offset_ms,
                CS.alca_engaged,
                radar_state
            )

            # Send pedal commands at 50Hz
            if (accel_needed > -1) and (accel_idx > -1) and frame % 2 == 0:
                messages.append(
                    self.tesla_can.create_pedal_command_msg(
                        apply_accel, int(accel_needed), accel_idx, self.pedalcan
                    )
                )
                self.PCC.pedal_idx = (self.PCC.pedal_idx + 1) % 16

            # Send iBooster commands at 10Hz
            if self.has_ibooster_ecu and frame % 10 == 0:
                messages.append(
                    self.tesla_can.create_ibst_command(
                        enabled, 15 * self.apply_brake, self.ibooster_idx,
                        CAN_CHASSIS[self.CP.carFingerprint]
                    )
                )
                self.ibooster_idx = (self.ibooster_idx + 1) % 16

            # AP1 long control when autopilot is disabled
            if self.autopilot_disabled:
                target_speed_kph = target_speed * CV.MS_TO_KPH
                if enabled:
                    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW1:
                        messages.append(self.tesla_can.create_ap1_long_control(
                            not CS.carNotInDrive, False, CS.pcc_enabled, target_speed_kph,
                            tesla_accel_limits, tesla_jerk_limits,
                            CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter
                        ))
                else:
                    if CS.carNotInDrive:
                        CS.cc_state = 0
                    else:
                        CS.cc_state = 1
                    # Send these values so we can enable at 0 km/h
                    default_accel_limits = [-1.4000000000000004, 1.8000000000000007]
                    default_jerk_limits = [-0.46000000000000085, 0.47600000000000003]
                    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW1:
                        messages.append(self.tesla_can.create_ap1_long_control(
                            not CS.carNotInDrive, False, False, 0,
                            default_accel_limits, default_jerk_limits,
                            CAN_POWERTRAIN[self.CP.carFingerprint], self.long_control_counter
                        ))

        return messages
