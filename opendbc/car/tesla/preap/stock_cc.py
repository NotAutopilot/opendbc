"""No-pedal stock-cruise transaction. Independent of MADS and of pedal long."""
from __future__ import annotations

from opendbc.car import structs
from opendbc.car.tesla.preap.constants import (
  STALK_DOUBLE_PULL_MS,
  STOCK_CC_CANCEL_DELAY_FRAMES,
  STOCK_CC_CANCEL_ECHO_MS,
  STOCK_CC_CONFIRM_MS,
  STOCK_CC_ENGAGE_TIMEOUT_FRAMES,
  STOCK_CC_SECOND_PULL_TIMEOUT_MS,
  STOCK_CC_SPOOF_ECHO_MS,
  STOCK_CC_TX_PERIOD_FRAMES,
  STOCK_CC_TX_TIMEOUT_MS,
)
from opendbc.car.tesla.values import CruiseButtons

StockCcState = structs.CarStateSP.PreapStockCcTransactionState

_UINT32_MASK = 0xFFFFFFFF

CANCEL = CruiseButtons.CANCEL
SET_ACCEL = CruiseButtons.SET_ACCEL
MAIN = CruiseButtons.MAIN
IDLE = CruiseButtons.IDLE
PASSTHROUGH_LEVERS = (
  CruiseButtons.RES_ACCEL,
  CruiseButtons.RES_ACCEL_2ND,
  CruiseButtons.DECEL_SET,
  CruiseButtons.DECEL_2ND,
)


def _elapsed_ms(now_ms: int, start_ms: int | None) -> int | None:
  if start_ms is None:
    return None
  return (int(now_ms) - int(start_ms)) & _UINT32_MASK


def _not_before(now_ms: int, origin_ms: int | None) -> bool:
  elapsed = _elapsed_ms(now_ms, origin_ms)
  return elapsed is not None and elapsed < (1 << 31)


def _generation_newer(candidate: int, bound: int | None) -> bool:
  if bound is None:
    return False
  delta = (int(candidate) - int(bound)) & _UINT32_MASK
  return 0 < delta < (1 << 31)


class StockCcTransaction:
  """Canonical no-pedal 0x45 cancel/re-engage handshake.

  Panda independently validates the same physical events and exact TX tuples.
  Host events never authorize Panda.
  """

  def __init__(self, active: bool):
    self.active = bool(active)
    self.state = StockCcState.idle
    self.bound_counter = 0
    self.host_di_confirmed = False
    self.enable_pending = False
    self.live_stw: dict[str, int] | None = None
    self._stalk_counter: int | None = None
    self._stalk_armed = False
    self._prev_lever: int | None = None
    self._first_pull_ms: int | None = None
    self._pull2_latched = False
    self._cancel_request_ms: int | None = None
    self._cancel_request_frame: int | None = None
    self._cancel_sent = False
    self._cancel_sent_ms: int | None = None
    self._set_sent = False
    self._set_sent_ms: int | None = None
    self._set_request_ms: int | None = None
    self._post_cancel_di = False
    self._di_enabled = False
    self._di_generation = 0
    self._di_generation_accepted = False
    self._cancel_bound_generation = None
    self._set_bound_generation = None
    self._epoch = 0
    self._panda_seen_counter = 0
    self._panda_counter_at_bind: int | None = None
    self._panda_matched = False
    self._now_ms = 0
    self._blocked = False
    self._echo_lever: int | None = None
    self._echo_counter: int | None = None
    self._echo_ms: int | None = None
    self._echo_window_ms = 0
    self._need_release = False
    self._unpublished_terminal = False
    self._emitted_bound: int | None = None

  def publish(self, ret_sp: structs.CarStateSP) -> None:
    ret_sp.preapStockCcState = self.state
    ret_sp.preapStockCcBoundCounter = int(self.bound_counter) & 0xFF
    ret_sp.preapStockCcHostDiConfirmed = bool(self.host_di_confirmed)
    ret_sp.preapStockCcEnablePending = bool(self.enable_pending)

  def acknowledge_publication(self, ret_sp: structs.CarStateSP) -> None:
    # Consume the one-shot latch only after cancelledOrFailed is externally visible.
    if ret_sp.preapStockCcState == StockCcState.cancelledOrFailed:
      self._unpublished_terminal = False

  def bind_epoch(self, epoch: int, sequence: int) -> None:
    del sequence
    epoch = int(epoch)
    if epoch != 0 and self._epoch != 0 and epoch != self._epoch and self.state not in (
      StockCcState.idle, StockCcState.cancelledOrFailed,
    ):
      self._fail()
    self._epoch = epoch

  def is_echo(self, lever: int, counter: int, now_ms: int) -> bool:
    if self._echo_lever is None or self._echo_ms is None or self._echo_counter is None:
      return False
    elapsed = _elapsed_ms(now_ms, self._echo_ms)
    if elapsed is None or elapsed > self._echo_window_ms:
      self._echo_lever = None
      return False
    return int(lever) == self._echo_lever and (int(counter) & 0xF) == self._echo_counter

  def sync_counter(self, counter: int) -> None:
    self._stalk_counter = int(counter) & 0xF

  def update_live_stw(self, values: dict[str, int]) -> None:
    self.live_stw = dict(values)

  def update_health(self, *, blocked: bool, brake_pressed: bool = False) -> None:
    abort = bool(blocked) or bool(brake_pressed)
    # Brake or a required-source blocker revokes confirmed authority as well as
    # in-flight state. cancelledOrFailed is the existing non-authoritative sink.
    if abort and self.state not in (StockCcState.idle, StockCcState.cancelledOrFailed):
      self._fail()
    self._blocked = abort
    if abort:
      self._stalk_armed = False

  def update_di(self, enabled: bool, now_ms: int, generation: int | None = None) -> None:
    now_ms = int(now_ms) & _UINT32_MASK
    self._now_ms = now_ms
    # Generation-less samples cannot move the accepted DI baseline.
    if generation is None:
      return
    candidate = int(generation) & _UINT32_MASK
    # First non-None generation bootstraps the accepted baseline. After that,
    # only a strictly newer UInt32 sample may change level or drive edges.
    if self._di_generation_accepted and not _generation_newer(candidate, self._di_generation):
      return
    prev_enabled = self._di_enabled
    prev_generation = self._di_generation
    self._di_enabled = bool(enabled)
    self._di_generation = candidate
    self._di_generation_accepted = True
    if not self.active:
      return
    if (self.state == StockCcState.confirmed and prev_enabled and not self._di_enabled and
        _generation_newer(self._di_generation, prev_generation)):
      self._fail()
      return
    if (self._cancel_sent and not self._di_enabled and not self._post_cancel_di and
        _not_before(now_ms, self._cancel_sent_ms) and
        _generation_newer(self._di_generation, self._cancel_bound_generation)):
      elapsed = _elapsed_ms(now_ms, self._cancel_sent_ms)
      if elapsed is None or elapsed >= STOCK_CC_CONFIRM_MS:
        self._fail()
        return
      self._post_cancel_di = True
      if self.state == StockCcState.awaitingCancelConfirmation:
        if self._pull2_latched:
          self._enter_reengage(now_ms)
        else:
          self.state = StockCcState.awaitingSecondPull
    if (self._set_sent and self._di_enabled and _not_before(now_ms, self._set_sent_ms) and
        _generation_newer(self._di_generation, self._set_bound_generation)):
      self.host_di_confirmed = True
      self._try_confirm()

  def update_panda(self, *, counter: int | None, confirmed: bool, controls_allowed_longitudinal: bool) -> None:
    if counter is None:
      if self.active and self.state not in (StockCcState.idle, StockCcState.cancelledOrFailed):
        self._fail()
      return
    self._panda_seen_counter = int(counter) & 0xFF
    if not self.active:
      return
    if self.state not in (StockCcState.awaitingDiConfirmation, StockCcState.reengageRequested, StockCcState.confirmed):
      return
    if self._panda_counter_at_bind is None:
      if self.state == StockCcState.confirmed:
        self._fail()
      return
    expected = (self._panda_counter_at_bind + 1) & 0xFF
    self.bound_counter = expected
    live_counter = int(counter) & 0xFF
    if self.state == StockCcState.confirmed:
      if (not confirmed) or (not controls_allowed_longitudinal) or live_counter != expected:
        self._fail()
      return
    if self.state == StockCcState.awaitingDiConfirmation:
      if confirmed and live_counter != expected:
        self._fail()
        return
      if confirmed and controls_allowed_longitudinal and live_counter == expected:
        self._panda_matched = True
        self._try_confirm()

  def update_stalk(self, lever: int, counter: int, now_ms: int) -> None:
    now_ms = int(now_ms) & _UINT32_MASK
    self._now_ms = now_ms
    lever = int(lever)
    counter &= 0xF
    consecutive = self._stalk_counter is None or counter == ((self._stalk_counter + 1) & 0xF)
    self._stalk_counter = counter
    if not self.active:
      self._prev_lever = lever
      return
    if self._need_release:
      if consecutive and lever == IDLE:
        self._reset_to_idle()
      self._prev_lever = lever
      return
    if not consecutive:
      if self.state not in (StockCcState.idle, StockCcState.cancelledOrFailed):
        self._fail()
      self._stalk_armed = False
      self._first_pull_ms = None
      self._prev_lever = lever
      return
    if lever == CANCEL:
      if self._prev_lever != CANCEL and self.state not in (StockCcState.idle, StockCcState.cancelledOrFailed):
        self._fail()
      self._stalk_armed = False
      self._prev_lever = lever
      return
    if lever in PASSTHROUGH_LEVERS:
      self._stalk_armed = False
      self._prev_lever = lever
      return
    self._prev_lever = lever
    if self._blocked:
      self._stalk_armed = False
      return
    if lever == IDLE:
      self._stalk_armed = True
      return
    if lever != MAIN:
      self._stalk_armed = False
      return
    if not self._stalk_armed:
      return
    self._stalk_armed = False
    self._process_main_pull(now_ms)

  def tick_timeouts(self, now_ms: int) -> None:
    if not self.active or self.state in (StockCcState.idle, StockCcState.confirmed, StockCcState.cancelledOrFailed):
      return
    now_ms = int(now_ms) & _UINT32_MASK
    limits = {
      StockCcState.cancelRequested: (_elapsed_ms(now_ms, self._cancel_request_ms), STOCK_CC_TX_TIMEOUT_MS + STOCK_CC_CANCEL_DELAY_FRAMES * 10),
      StockCcState.awaitingCancelConfirmation: (_elapsed_ms(now_ms, self._cancel_sent_ms), STOCK_CC_CONFIRM_MS),
      StockCcState.awaitingSecondPull: (_elapsed_ms(now_ms, self._first_pull_ms), STOCK_CC_SECOND_PULL_TIMEOUT_MS),
      StockCcState.reengageRequested: (_elapsed_ms(now_ms, self._set_request_ms), STOCK_CC_ENGAGE_TIMEOUT_FRAMES * 10),
      StockCcState.awaitingDiConfirmation: (_elapsed_ms(now_ms, self._set_sent_ms), STOCK_CC_CONFIRM_MS),
    }
    elapsed, limit = limits.get(self.state, (None, None))
    if elapsed is not None and limit is not None and elapsed >= limit:
      self._fail()

  def poll_tx(self, frame: int) -> int | None:
    if not self.active or self.live_stw is None:
      return None
    frame = int(frame)
    if self.state == StockCcState.cancelRequested and not self._cancel_sent:
      if self._cancel_request_frame is None:
        self._cancel_request_frame = frame
      if (frame - self._cancel_request_frame) < STOCK_CC_CANCEL_DELAY_FRAMES:
        return None
      return CANCEL
    if self.state == StockCcState.reengageRequested and self._cancel_sent and self._post_cancel_di and not self._set_sent:
      if frame % STOCK_CC_TX_PERIOD_FRAMES != 0:
        return None
      return SET_ACCEL
    return None

  def note_tx(self, lever: int, counter: int, now_ms: int) -> None:
    now_ms = int(now_ms) & _UINT32_MASK
    lever = int(lever)
    counter &= 0xF
    self._stalk_counter = counter
    if lever == CANCEL:
      self._cancel_sent = True
      self._cancel_sent_ms = now_ms
      self._cancel_bound_generation = self._di_generation
      self._echo_lever = CANCEL
      self._echo_counter = counter
      self._echo_ms = now_ms
      self._echo_window_ms = STOCK_CC_CANCEL_ECHO_MS
      if self.state == StockCcState.cancelRequested:
        self.state = StockCcState.awaitingCancelConfirmation
    elif lever == SET_ACCEL:
      self._set_sent = True
      self._set_sent_ms = now_ms
      self._set_bound_generation = self._di_generation
      self._echo_lever = SET_ACCEL
      self._echo_counter = counter
      self._echo_ms = now_ms
      self._echo_window_ms = STOCK_CC_SPOOF_ECHO_MS
      if self._panda_counter_at_bind is None:
        self._panda_counter_at_bind = self._panda_seen_counter
      self.bound_counter = (self._panda_counter_at_bind + 1) & 0xFF
      if self.state == StockCcState.reengageRequested:
        self.state = StockCcState.awaitingDiConfirmation

  def tx_counter(self) -> int:
    if self._stalk_counter is not None:
      return (int(self._stalk_counter) + 1) & 0xF
    live = 0 if self.live_stw is None else int(self.live_stw.get("MC_STW_ACTN_RQ", 0))
    return (live + 1) % 16

  def _process_main_pull(self, now_ms: int) -> None:
    elapsed = _elapsed_ms(now_ms, self._first_pull_ms)
    is_second = elapsed is not None and 0 < elapsed < STALK_DOUBLE_PULL_MS
    if is_second:
      if self.state in (
        StockCcState.cancelRequested, StockCcState.awaitingCancelConfirmation, StockCcState.awaitingSecondPull,
      ):
        self._pull2_latched = True
        if self._cancel_sent and self._post_cancel_di:
          self._enter_reengage(now_ms)
      return
    self._first_pull_ms = now_ms
    self._pull2_latched = False
    self._request_cancel(now_ms)

  def _request_cancel(self, now_ms: int) -> None:
    if self.state == StockCcState.cancelledOrFailed:
      return
    self._cancel_request_ms = now_ms
    self._cancel_request_frame = None
    self._cancel_sent = False
    self._post_cancel_di = False
    self._cancel_bound_generation = None
    self._set_sent = False
    self._set_sent_ms = None
    self._set_bound_generation = None
    self._panda_matched = False
    self._panda_counter_at_bind = self._panda_seen_counter
    self.host_di_confirmed = False
    self.enable_pending = False
    self.state = StockCcState.cancelRequested

  def _enter_reengage(self, now_ms: int) -> None:
    if not (self._cancel_sent and self._post_cancel_di):
      return
    self._set_request_ms = now_ms
    self._panda_counter_at_bind = self._panda_seen_counter
    self.bound_counter = (self._panda_counter_at_bind + 1) & 0xFF
    self.state = StockCcState.reengageRequested

  def _try_confirm(self) -> None:
    if self.state != StockCcState.awaitingDiConfirmation:
      return
    if self.host_di_confirmed and self._panda_matched:
      self.state = StockCcState.confirmed
      self.enable_pending = True

  def _fail(self) -> None:
    self.state = StockCcState.cancelledOrFailed
    self.enable_pending = False
    self.host_di_confirmed = False
    self._pull2_latched = False
    self._cancel_sent = False
    self._set_sent = False
    self._post_cancel_di = False
    self._cancel_bound_generation = None
    self._set_bound_generation = None
    self._panda_matched = False
    self._first_pull_ms = None
    self._stalk_armed = False
    self._need_release = True
    self._unpublished_terminal = True

  def _reset_to_idle(self) -> None:
    if self._unpublished_terminal:
      return
    self.state = StockCcState.idle
    self.host_di_confirmed = False
    self.enable_pending = False
    self._stalk_armed = False
    self._first_pull_ms = None
    self._pull2_latched = False
    self._cancel_request_ms = None
    self._cancel_request_frame = None
    self._cancel_sent = False
    self._cancel_sent_ms = None
    self._post_cancel_di = False
    self._cancel_bound_generation = None
    self._set_sent = False
    self._set_sent_ms = None
    self._set_bound_generation = None
    self._set_request_ms = None
    self._panda_matched = False
    self._need_release = False
    self._echo_lever = None
    self._panda_counter_at_bind = None
