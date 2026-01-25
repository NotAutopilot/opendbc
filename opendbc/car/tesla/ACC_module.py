"""
NAP Adaptive Cruise Controller (ACC) Module

Controls adaptive cruise when pedal interceptor is not available.
Uses virtual stalk button presses to adjust Tesla's stock cruise control.

Migrated from Tinkla with NAP-specific parameter names and event types.
"""
import sys
import time
import cereal.messaging as messaging
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CruiseButtons, CruiseState
from opendbc.car.tesla.speed_utils.fleet_speed import FleetSpeed
from cereal import car


def get_cc_units_kph(is_imperial_units: bool) -> tuple[float, float]:
    """
    Get cruise control adjustment units based on imperial/metric setting.

    Returns (half_press_kph, full_press_kph) - the speed adjustments for
    single and double button presses.
    """
    if is_imperial_units:
        # Imperial unit cars adjust cruise in units of 1 and 5 mph
        half_press_kph = 1 * CV.MPH_TO_KPH
        full_press_kph = 5 * CV.MPH_TO_KPH
    else:
        # Metric cars adjust cruise in units of 1 and 5 kph
        half_press_kph = 1
        full_press_kph = 5
    return half_press_kph, full_press_kph


class ACCState:
    """Possible states of the ACC system, following DI_cruiseState naming scheme"""
    OFF = 0       # Disabled by UI
    STANDBY = 1   # Ready to be engaged
    ENABLED = 2   # Engaged
    NOT_READY = 9  # Not ready due to car state


def _current_time_millis():
    return int(round(time.time() * 1000))


class ACCController:
    """
    Adaptive Cruise Controller for Tesla.

    Used when pedal interceptor is not available. Controls Tesla's stock
    cruise control via virtual stalk button presses.
    """

    # Tesla cruise only functions above 17 MPH
    MIN_CRUISE_SPEED_MS = 17.1 * CV.MPH_TO_MS

    def __init__(self, long_controller):
        self.LongCtr = long_controller
        self.human_cruise_action_time = 0
        self.automated_cruise_action_time = 0
        self.radarState = messaging.sub_sock("radarState", conflate=True)
        self.last_update_time = 0

        # ACC state
        self.enable_adaptive_cruise = False
        self.prev_enable_adaptive_cruise = False
        self.autoresume = False
        self.adaptive = False

        # Input tracking
        self.last_brake_press_time = 0
        self.last_cruise_stalk_pull_time = 0
        self.prev_cruise_buttons = CruiseButtons.IDLE
        self.prev_cruise_state = 0

        # Speed control
        self.acc_speed_kph = 0.0
        self.speed_limit_kph = 0.0
        self.prev_speed_limit_kph = 0.0

        # State tracking
        self.user_has_braked = False
        self.has_gone_below_min_speed = False
        self.fast_decel_time = 0
        self.lead_last_seen_time_ms = 0

        # Fleet speed averaging
        average_speed_over_x_suggestions = 20  # 1 second (20x a second)
        self.fleet_speed = FleetSpeed(average_speed_over_x_suggestions)

    def update_stat(self, CS, enabled):
        """
        Update ACC state based on car state and user inputs.

        This method handles engagement/disengagement logic via the cruise stalk.
        """
        # Check if the cruise stalk was double pulled
        self.prev_enable_adaptive_cruise = self.enable_adaptive_cruise
        self.autoresume = CS.autoresumeAcc
        curr_time_ms = _current_time_millis()

        if not self.LongCtr.CP.openpilotLongitudinalControl:
            return
        if CS.enablePedal:
            return

        # Handle pressing the enable button
        if (
            CS.cruise_buttons == CruiseButtons.MAIN
            or (
                CS.cruise_buttons == CruiseButtons.DECEL_SET
                and not self.enable_adaptive_cruise
            )
        ) and self.prev_cruise_buttons != CS.cruise_buttons:
            double_pull = curr_time_ms - self.last_cruise_stalk_pull_time < 750
            self.last_cruise_stalk_pull_time = curr_time_ms

            ready = (
                (
                    enabled
                    or (
                        CS.cruise_buttons == CruiseButtons.DECEL_SET
                        and not self.enable_adaptive_cruise
                    )
                    or (
                        CS.cruise_buttons == CruiseButtons.MAIN
                        and CS.enableJustCC
                        and not self.enable_adaptive_cruise
                    )
                )
                and CruiseState.is_enabled_or_standby(CS.cruise_state)
                and CS.out.vEgo > self.MIN_CRUISE_SPEED_MS
            )

            if (
                ready
                and double_pull
                and (
                    CS.cruise_buttons == CruiseButtons.MAIN
                    or (
                        CS.cruise_buttons == CruiseButtons.DECEL_SET
                        and not self.enable_adaptive_cruise
                    )
                )
            ):
                # Decide adaptive or plain CC
                if CS.cruise_buttons == CruiseButtons.MAIN:
                    self.adaptive = not CS.enableJustCC
                else:
                    self.adaptive = False

                # A double pull enables ACC
                if not self.enable_adaptive_cruise:
                    if self.adaptive:
                        CS.longCtrlEvent = car.CarEvent.EventName.napAdaptiveCruiseEnabled
                    else:
                        CS.longCtrlEvent = car.CarEvent.EventName.napCruiseEnabled

                    self.enable_adaptive_cruise = True

                # Increase ACC speed to match current
                if self.adaptive:
                    self.acc_speed_kph = max(
                        CS.out.vEgoRaw * CV.MS_TO_KPH, self.speed_limit_kph
                    )
                else:
                    self.acc_speed_kph = CS.out.vEgoRaw * CV.MS_TO_KPH

                self.user_has_braked = False
                self.has_gone_below_min_speed = False
            else:
                # A single pull disables ACC (falling back to just steering)
                if CS.cruise_buttons == CruiseButtons.MAIN:
                    if self.enable_adaptive_cruise:
                        if self.adaptive:
                            CS.longCtrlEvent = car.CarEvent.EventName.napAdaptiveCruiseDisabled
                        else:
                            CS.longCtrlEvent = car.CarEvent.EventName.napCruiseDisabled
                        self.enable_adaptive_cruise = False

        # Handle pressing the cancel button
        if CS.cruise_buttons == CruiseButtons.CANCEL:
            if self.enable_adaptive_cruise:
                if self.adaptive:
                    CS.longCtrlEvent = car.CarEvent.EventName.napAdaptiveCruiseDisabled
                else:
                    CS.longCtrlEvent = car.CarEvent.EventName.napCruiseDisabled
                self.enable_adaptive_cruise = False
            self.acc_speed_kph = 0.0
            self.last_cruise_stalk_pull_time = 0

        # Handle pressing up and down buttons
        elif (
            CS.cruise_buttons != CruiseButtons.MAIN
            and self.enable_adaptive_cruise
            and CS.cruise_buttons != self.prev_cruise_buttons
        ):
            self._update_max_acc_speed(CS)

        # Handle brake press
        if CS.realBrakePressed:
            self.user_has_braked = True
            self.last_brake_press_time = _current_time_millis()
            if not self.autoresume:
                if self.enable_adaptive_cruise:
                    self.enable_adaptive_cruise = False
                    if self.adaptive:
                        CS.longCtrlEvent = car.CarEvent.EventName.napAdaptiveCruiseDisabled
                    else:
                        CS.longCtrlEvent = car.CarEvent.EventName.napCruiseDisabled

        if CS.out.vEgo < self.MIN_CRUISE_SPEED_MS:
            self.has_gone_below_min_speed = True

        # If autoresume is not enabled and not in standard CC, disable if braked or below min speed
        if not self.autoresume:
            if (
                (self.adaptive and not enabled)
                or self.user_has_braked
                or self.has_gone_below_min_speed
            ):
                if self.enable_adaptive_cruise:
                    self.enable_adaptive_cruise = False
                    CS.longCtrlEvent = car.CarEvent.EventName.napAdaptiveCruiseDisabled

        # Update prev state
        self.prev_cruise_buttons = CS.cruise_buttons
        self.prev_cruise_state = CS.cruise_state

    def _update_max_acc_speed(self, CS):
        """Adjust max ACC speed based on user cruise stalk actions"""
        half_press_kph, full_press_kph = get_cc_units_kph(CS.speed_units == "MPH")
        speed_change_map = {
            CruiseButtons.RES_ACCEL: half_press_kph,
            CruiseButtons.RES_ACCEL_2ND: full_press_kph,
            CruiseButtons.DECEL_SET: -1 * half_press_kph,
            CruiseButtons.DECEL_2ND: -1 * full_press_kph,
        }
        self.acc_speed_kph += speed_change_map.get(CS.cruise_buttons, 0)

        # Clip ACC speed between 0 and 170 KPH
        self.acc_speed_kph = min(self.acc_speed_kph, 170)
        self.acc_speed_kph = max(self.acc_speed_kph, 0)

    def update_acc(
        self,
        enabled,
        CS,
        frame,
        actuators,
        pcm_speed,
        speed_limit_kph,
        set_speed_limit_active,
        speed_limit_offset,
    ):
        """
        Decide which cruise control button to simulate.

        Returns the CruiseButton to press, or None.
        """
        # Update speed limit state
        self.prev_speed_limit_kph = self.speed_limit_kph
        if set_speed_limit_active and speed_limit_kph > 0:
            self.speed_limit_kph = speed_limit_kph + speed_limit_offset
            if int(self.prev_speed_limit_kph) != int(self.speed_limit_kph):
                self.acc_speed_kph = self.speed_limit_kph
                self.fleet_speed.reset_averager()
        else:
            self.speed_limit_kph = 0.0

        current_time_ms = _current_time_millis()
        if CruiseButtons.should_be_throttled(CS.cruise_buttons):
            self.human_cruise_action_time = current_time_ms

        button_to_press = None

        # If ACC is disabled, disengage traditional cruise control
        if (
            self.prev_enable_adaptive_cruise
            and not self.enable_adaptive_cruise
            and CS.cruise_state == CruiseState.ENABLED
        ):
            button_to_press = CruiseButtons.CANCEL

        # If non-adaptive and we just engaged ACC but pcm is not engaged, engage it
        if (
            not self.adaptive
            and self.enable_adaptive_cruise
            and CS.cruise_state != CruiseState.ENABLED
        ):
            button_to_press = CruiseButtons.MAIN

        # If plain CC (not adaptive), just return None or Cancel
        if not self.adaptive:
            self.acc_speed_kph = CS.v_cruise_actual
            return button_to_press

        # Disengage if cruise is canceled
        if (
            not self.enable_adaptive_cruise
            and CS.cruise_state >= 2
            and CS.cruise_state <= 4
        ):
            return CruiseButtons.CANCEL

        # Check for lead vehicle
        lead_1 = None
        lead = messaging.recv_one_or_none(self.radarState)
        if lead is not None:
            lead_1 = lead.radarState.leadOne
            if lead_1.dRel:
                self.lead_last_seen_time_ms = current_time_ms

        if self.enable_adaptive_cruise and enabled and pcm_speed is not None:
            button_to_press = self._calc_button(CS, pcm_speed)

        if button_to_press:
            self.automated_cruise_action_time = current_time_ms
            # If trying to slow below the min cruise speed, just cancel cruise
            if (
                CruiseButtons.is_decel(button_to_press)
                and CS.v_cruise_actual - 1 < self.MIN_CRUISE_SPEED_MS * CV.MS_TO_KPH
            ):
                button_to_press = CruiseButtons.CANCEL
            if button_to_press == CruiseButtons.CANCEL:
                self.fast_decel_time = current_time_ms

        return button_to_press

    def _should_autoengage_cc(self, CS, lead_car=None):
        """
        Determine if cruise control should be automatically (re)engaged.

        Conditions:
        1. The carstate allows cruise control
        2. There is no imminent threat of collision
        3. The user did not cancel ACC by pressing the brake
        """
        cruise_ready = (
            self.enable_adaptive_cruise
            and CS.cruise_state == CruiseState.STANDBY
            and CS.out.vEgo >= self.MIN_CRUISE_SPEED_MS
            and _current_time_millis() > self.fast_decel_time + 2000
        )

        slow_lead = (
            lead_car
            and lead_car.dRel > 0
            and lead_car.vRel < 0
            or self._fast_decel_required(CS, lead_car)
        )

        # "Autoresume" mode allows cruise to engage even after brake events
        autoresume_ready = (
            self.autoresume
            and CS.out.aEgo >= 0.1
            and not CS.HSO.human_control
            and _current_time_millis() > self.last_brake_press_time + 1000
        )

        braked = self.user_has_braked or self.has_gone_below_min_speed

        return cruise_ready and not slow_lead and (autoresume_ready or not braked)

    def _fast_decel_required(self, CS, lead_car):
        """Identify situations requiring rapid deceleration"""
        if not lead_car or not lead_car.dRel:
            return False

        collision_imminent = self._seconds_to_collision(CS, lead_car) < 4
        lead_absolute_speed_ms = lead_car.vRel + CS.out.vEgo
        lead_too_slow = lead_absolute_speed_ms < self.MIN_CRUISE_SPEED_MS

        return collision_imminent or lead_too_slow

    def _seconds_to_collision(self, CS, lead_car):
        """Calculate time to collision with lead vehicle"""
        if not lead_car or not lead_car.dRel:
            return sys.maxsize
        elif lead_car.vRel >= 0:
            return sys.maxsize
        return abs(float(lead_car.dRel) / lead_car.vRel)

    def _calc_button(self, CS, desired_speed_ms):
        """
        Calculate which button to press based on desired speed.

        Returns CruiseButton or None.
        """
        button_to_press = None

        # Automatically engage traditional cruise if appropriate
        if self._should_autoengage_cc(CS) and desired_speed_ms >= CS.out.vEgo:
            button_to_press = CruiseButtons.RES_ACCEL

        # If traditional cruise is engaged, then control it
        elif (
            CS.cruise_state == CruiseState.ENABLED
            and self._no_human_action_for(milliseconds=3000)
            and self._no_automated_action_for(milliseconds=400)
        ):
            # Difference between OP's target speed and current cruise speed
            speed_offset_kph = desired_speed_ms * CV.MS_TO_KPH - CS.v_cruise_actual

            half_press_kph, full_press_kph = get_cc_units_kph(CS.speed_units == "MPH")

            # Reduce cruise speed significantly if necessary
            if desired_speed_ms < self.MIN_CRUISE_SPEED_MS:
                button_to_press = CruiseButtons.CANCEL
            if speed_offset_kph < -2 * full_press_kph and CS.v_cruise_actual > 0:
                button_to_press = CruiseButtons.CANCEL
            elif speed_offset_kph < -0.6 * full_press_kph and CS.v_cruise_actual > 0:
                button_to_press = CruiseButtons.DECEL_2ND
            elif speed_offset_kph < -0.9 * half_press_kph and CS.v_cruise_actual > 0:
                button_to_press = CruiseButtons.DECEL_SET
            # Increase cruise speed if possible
            elif CS.out.vEgo > self.MIN_CRUISE_SPEED_MS:
                available_speed_kph = self.acc_speed_kph - CS.v_cruise_actual
                if (
                    speed_offset_kph >= full_press_kph
                    and full_press_kph < available_speed_kph
                ):
                    button_to_press = CruiseButtons.RES_ACCEL_2ND
                elif (
                    speed_offset_kph >= half_press_kph
                    and half_press_kph < available_speed_kph
                ):
                    button_to_press = CruiseButtons.RES_ACCEL

        return button_to_press

    def _no_human_action_for(self, milliseconds):
        return _current_time_millis() > self.human_cruise_action_time + milliseconds

    def _no_automated_action_for(self, milliseconds):
        return _current_time_millis() > self.automated_cruise_action_time + milliseconds

    def _no_action_for(self, milliseconds):
        return self._no_human_action_for(milliseconds) and self._no_automated_action_for(milliseconds)
