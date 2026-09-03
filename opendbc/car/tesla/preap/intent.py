"""Pre-AP intent translator. Not an engagement owner."""
from __future__ import annotations

from dataclasses import dataclass

from opendbc.car import structs
from opendbc.car.tesla.preap.constants import STALK_DOUBLE_PULL_MS
from opendbc.car.tesla.values import CruiseButtons

_UINT32_MASK = 0xFFFFFFFF

LateralIntent = structs.CarStateSP.PreapLateralIntent
LongitudinalIntent = structs.CarStateSP.PreapLongitudinalIntent
EngagementMode = structs.CarParamsSP.PreapLateralEngagementMode

_VALID_MODES = (
  EngagementMode.independent,
  EngagementMode.cruiseCoupled,
  EngagementMode.longitudinalOnly,
)

# Direct stock-cruise +/- . Same four RX levers as StockCcTransaction/Panda:
# disarm the instantaneous edge, keep the pending double-pull origin.
_PASSTHROUGH_LEVERS = (
  CruiseButtons.RES_ACCEL,
  CruiseButtons.RES_ACCEL_2ND,
  CruiseButtons.DECEL_SET,
  CruiseButtons.DECEL_2ND,
)


@dataclass(frozen=True)
class PreAPIntentRecord:
  lateral: LateralIntent = LateralIntent.none
  longitudinal: LongitudinalIntent = LongitudinalIntent.none
  sequence: int = 0


class PreAPIntentTranslator:
  """Trusted-stalk / terminal-action producer for the CarStateSP intent outbox.

  MADS remains the sole lateral owner. This module only latches atomic records.
  """

  def __init__(self, mode: EngagementMode):
    self.mode = mode if mode in _VALID_MODES else None
    self.record = PreAPIntentRecord()
    self.long_active: bool | None = None
    self._stalk_counter: int | None = None
    self._stalk_armed = False
    self._prev_lever: int | None = None
    self._first_pull_ms: int | None = None
    self._brake_pressed = False
    self._epas_fault = False
    # Unknown sources start blocked so boot cannot emit a disable record.
    self._terminal_failure = False
    self._blocked = True
    self.stock_cc_active = False
    self._coupled_deferred = False
    # Driver long intent. Survives gas override; interceptor authority does not.
    self.enable_long_control = False
    self._gas_pressed = False
    # True when panda would accept ENABLE (STATE==NO_FAULT). STATE 4/5 may stay
    # PedalFeedback.available for health but must not arm Pedal Cruise Engaged.
    self._interceptor_no_fault = True
    # Second pull while panda would refuse (gas down and/or not NO_FAULT).
    self._enable_blocked_by_gas = False

  def set_long_active(self, long_active: bool) -> None:
    """Controller feeds prior-cycle logical standard-long active state.

    No-pedal stock cruise uses CC.enabled. Pedal/openpilot-long uses CC.longActive.
    Accurate classification is not available from CAN at CarState. Do not infer
    pedal/stock long from factual DI cruiseState.enabled. One-cycle lag is
    expected. Until the first explicit input, pulls fail closed.
    """
    self.long_active = bool(long_active)

  def _publish(self, lateral: LateralIntent, longitudinal: LongitudinalIntent) -> None:
    sequence = (self.record.sequence + 1) & _UINT32_MASK
    self.record = PreAPIntentRecord(lateral, longitudinal, sequence)
    if longitudinal == LongitudinalIntent.enable:
      self.enable_long_control = True
    elif longitudinal == LongitudinalIntent.disable:
      self.enable_long_control = False

  def _clear_pending(self) -> None:
    self._first_pull_ms = None
    self._stalk_armed = False
    self._coupled_deferred = False
    self._enable_blocked_by_gas = False

  def _disable(self, *, coupled_only: bool = False) -> None:
    if coupled_only:
      lateral = LateralIntent.forceDisable if self.mode == EngagementMode.cruiseCoupled else LateralIntent.none
    else:
      lateral = LateralIntent.forceDisable
    self._publish(lateral, LongitudinalIntent.disable)
    self._clear_pending()

  def update_health(self, *, blocked: bool, epas_fault: bool, brake_pressed: bool,
                    gas_pressed: bool = False, interceptor_no_fault: bool = True) -> None:
    if (epas_fault and not self._epas_fault) or (blocked and not self._blocked):
      self._disable()
    elif brake_pressed and not self._brake_pressed and self.long_active:
      self._disable(coupled_only=True)

    self._epas_fault = epas_fault
    self._brake_pressed = brake_pressed
    self._blocked = blocked
    if blocked or epas_fault:
      self._stalk_armed = False
      self._enable_blocked_by_gas = False

    gas_released = self._gas_pressed and not gas_pressed
    self._gas_pressed = bool(gas_pressed)
    no_fault_restored = (not self._interceptor_no_fault) and bool(interceptor_no_fault)
    self._interceptor_no_fault = bool(interceptor_no_fault)
    # Deferred enable after a panda-refuse block: only fire when gas is up AND
    # interceptor is NO_FAULT. Do not latch Pedal Cruise Engaged on STATE 4/5.
    if self._enable_blocked_by_gas and (gas_released or no_fault_restored):
      if (not self._gas_pressed) and self._interceptor_no_fault and not (blocked or epas_fault or brake_pressed):
        self._enable_blocked_by_gas = False
        self._publish(LateralIntent.none, LongitudinalIntent.enable)

  def update_terminal_failure(self, failed: bool) -> None:
    """Publish an attributable long terminal exit once per failure edge."""
    if failed and not self._terminal_failure:
      self._coupled_deferred = False
      self._disable(coupled_only=True)
    self._terminal_failure = failed

  def publish_confirmed_coupled_enable(self) -> None:
    """Release deferred cruiseCoupled enable on the matching StockCC confirmation."""
    if not self._coupled_deferred or self.mode != EngagementMode.cruiseCoupled:
      return
    self._coupled_deferred = False
    self._publish(LateralIntent.mainCruiseRequest, LongitudinalIntent.enable)

  def update_stalk(self, lever: int, counter: int, now_ms: int) -> None:
    lever = int(lever)
    counter &= 0xF
    consecutive = self._stalk_counter is None or counter == ((self._stalk_counter + 1) & 0xF)
    self._stalk_counter = counter
    if not consecutive:
      # Held/echo/resync: drop arm and pending; require a fresh trusted idle.
      self._clear_pending()
      self._prev_lever = lever
      return

    if lever == CruiseButtons.CANCEL:
      if self._prev_lever != CruiseButtons.CANCEL:
        self._disable()
      self._stalk_armed = False
      self._prev_lever = lever
      return

    self._prev_lever = lever
    if self._blocked or self._epas_fault or self.mode is None:
      self._stalk_armed = False
      return

    if lever == CruiseButtons.IDLE:
      self._stalk_armed = True
      return

    if lever in _PASSTHROUGH_LEVERS:
      # Direct stock-cruise +/- : disarm the edge, keep pending double-pull origin.
      self._stalk_armed = False
      return

    if lever != CruiseButtons.MAIN:
      self._stalk_armed = False
      self._first_pull_ms = None
      return

    # Held MAIN cannot re-fire: a pull consumes the idle arm.
    if not self._stalk_armed:
      return
    self._stalk_armed = False
    self._process_main_pull(now_ms & _UINT32_MASK)

  def _process_main_pull(self, now_ms: int) -> None:
    if self.mode is None:
      return
    elapsed = None if self._first_pull_ms is None else (now_ms - self._first_pull_ms) & _UINT32_MASK
    if elapsed is not None and 0 < elapsed < STALK_DOUBLE_PULL_MS:
      self._first_pull_ms = None
      if self.stock_cc_active and self.mode == EngagementMode.cruiseCoupled:
        self._coupled_deferred = True
        self._publish(LateralIntent.none, LongitudinalIntent.none)
        return
      if self._gas_pressed or not self._interceptor_no_fault:
        # Panda will not grant controlsAllowed while interceptor gas is down or
        # feedback is not NO_FAULT. STATE 4/5 may stay PedalFeedback.available
        # (health) but must not alone arm Pedal Cruise Engaged / CC.enabled /
        # longActive (controlsMismatch 0000000c). Do not read live ca/caLong.
        self._enable_blocked_by_gas = True
        if self.mode == EngagementMode.cruiseCoupled:
          self._publish(LateralIntent.mainCruiseRequest, LongitudinalIntent.none)
        return
      lateral = LateralIntent.mainCruiseRequest if self.mode == EngagementMode.cruiseCoupled else LateralIntent.none
      self._publish(lateral, LongitudinalIntent.enable)
      return

    self._first_pull_ms = now_ms
    if self.long_active:
      lateral = LateralIntent.forceDisable if self.mode == EngagementMode.cruiseCoupled else LateralIntent.none
      self._publish(lateral, LongitudinalIntent.disable)
    elif self.long_active is None:
      self._disable()
    elif self.mode == EngagementMode.independent:
      self._publish(LateralIntent.mainCruiseRequest, LongitudinalIntent.none)
    else:
      self._publish(LateralIntent.none, LongitudinalIntent.none)
