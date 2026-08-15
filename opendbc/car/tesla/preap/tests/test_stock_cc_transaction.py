import unittest

from opendbc.car import structs
from opendbc.car.tesla.preap.constants import (
  STALK_DOUBLE_PULL_MS,
  STOCK_CC_CANCEL_DELAY_FRAMES,
  STOCK_CC_TX_PERIOD_FRAMES,
)
from opendbc.car.tesla.preap.carstate import REQUIRED_SOURCE_KEYS, required_sources_fresh
from opendbc.car.tesla.preap.stock_cc import PASSTHROUGH_LEVERS, StockCcTransaction
from opendbc.car.tesla.preap.teslacan import STW_DEFAULTS
from opendbc.car.tesla.values import CruiseButtons

State = structs.CarStateSP.PreapStockCcTransactionState
CANCEL = CruiseButtons.CANCEL
SET_ACCEL = CruiseButtons.SET_ACCEL
MAIN = CruiseButtons.MAIN
IDLE = CruiseButtons.IDLE


def _live(counter=0, wiper=2):
  values = dict(STW_DEFAULTS)
  values["MC_STW_ACTN_RQ"] = counter
  values["WprSw6Posn"] = wiper
  values["DTR_Dist_Rq"] = 255
  return values


class _Txn:
  def __init__(self, active=True):
    self.t = StockCcTransaction(active)
    self.t.update_live_stw(_live())
    self.now = 0
    self.counter = 0
    self.frame = 0
    self.gen = 0

  def publish(self):
    sp = structs.CarStateSP()
    self.t.publish(sp)
    self.t.acknowledge_publication(sp)
    return sp

  def stalk(self, lever, now=None):
    if now is not None:
      self.now = now
    self.t.update_live_stw(_live(self.counter))
    self.t.update_stalk(lever, self.counter, self.now)
    sent = self.counter
    self.counter = (self.counter + 1) & 0xF
    return sent

  def pull(self, now=None):
    if now is not None:
      self.now = now
    self.stalk(IDLE)
    self.stalk(MAIN)

  def tick(self, now=None):
    if now is not None:
      self.now = now
    self.t.tick_timeouts(self.now)

  def di(self, enabled, now=None, generation=None):
    if now is not None:
      self.now = now
    if generation is None:
      self.gen = (self.gen + 1) & 0xFFFFFFFF
      generation = self.gen
    else:
      self.gen = int(generation) & 0xFFFFFFFF
    self.t.update_di(enabled, self.now, generation)

  def panda(self, counter, confirmed, allowed=True):
    self.t.update_panda(counter=counter, confirmed=confirmed, controls_allowed_longitudinal=allowed)

  def tx(self, frames=40):
    sent = []
    for _ in range(frames):
      lever = self.t.poll_tx(self.frame)
      self.frame += 1
      if lever is not None:
        counter = self.t.tx_counter()
        self.t.note_tx(lever, counter, self.now)
        self.counter = (counter + 1) & 0xF
        sent.append((lever, counter))
        return sent[-1]
    return None

  def cancel_and_wait_di(self, now=0):
    self.pull(now)
    self.assert_state(State.cancelRequested)
    pair = self.tx()
    self.assertIsNotNone(pair)
    self.assertEqual(pair[0], CANCEL)
    self.assert_state(State.awaitingCancelConfirmation)
    self.now = now + 20
    self.di(False)
    return pair

  def assert_state(self, state):
    if self.t.state != state:
      raise AssertionError(f"{self.t.state} != {state}")

  def assertIsNotNone(self, value):
    if value is None:
      raise AssertionError("unexpected None")

  def assertEqual(self, left, right):
    if left != right:
      raise AssertionError(f"{left} != {right}")


class TestStockCcTransaction(unittest.TestCase):
  def test_inactive_never_leaves_idle(self):
    h = _Txn(active=False)
    h.pull(0)
    h.tx()
    h.di(False)
    self.assertEqual(h.t.state, State.idle)
    self.assertFalse(h.publish().preapStockCcEnablePending)

  def test_every_canonical_state_and_prompt_cancel(self):
    h = _Txn()
    self.assertEqual(h.t.state, State.idle)
    h.pull(0)
    self.assertEqual(h.t.state, State.cancelRequested)
    self.assertIsNone(h.t.poll_tx(0))
    self.assertIsNone(h.t.poll_tx(STOCK_CC_CANCEL_DELAY_FRAMES - 1))
    pair = h.tx()
    self.assertEqual(pair[0], CANCEL)
    self.assertEqual(h.t.state, State.awaitingCancelConfirmation)
    h.di(False, 30)
    self.assertEqual(h.t.state, State.awaitingSecondPull)
    h.pull(399)
    self.assertEqual(h.t.state, State.reengageRequested)
    set_pair = h.tx()
    self.assertEqual(set_pair[0], SET_ACCEL)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    h.di(True, 420)
    h.panda(1, True, True)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t.enable_pending)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.confirmed)
    self.assertTrue(sp.preapStockCcEnablePending)
    self.assertTrue(sp.preapStockCcHostDiConfirmed)
    self.assertEqual(sp.preapStockCcBoundCounter, 1)

  def test_early_pull2_latches_before_cancel_and_blocks_set(self):
    h = _Txn()
    h.pull(0)
    h.pull(50)
    self.assertEqual(h.t.state, State.cancelRequested)
    self.assertTrue(h.t._pull2_latched)
    self.assertIsNone(h.t.poll_tx(5))
    pair = h.tx()
    self.assertEqual(pair[0], CANCEL)
    self.assertNotEqual(h.t.state, State.reengageRequested)
    h.di(False, 80)
    self.assertEqual(h.t.state, State.reengageRequested)
    set_pair = h.tx()
    self.assertEqual(set_pair[0], SET_ACCEL)

  def test_strict_399_400_401_window(self):
    for delta, expect_set in ((399, True), (400, False), (401, False)):
      with self.subTest(delta=delta):
        h = _Txn()
        h.cancel_and_wait_di(0)
        self.assertEqual(h.t.state, State.awaitingSecondPull)
        h.pull(delta)
        if expect_set:
          self.assertEqual(h.t.state, State.reengageRequested)
          self.assertEqual(h.tx()[0], SET_ACCEL)
        else:
          self.assertEqual(h.t.state, State.cancelRequested)
          self.assertEqual(h.tx()[0], CANCEL)

  def test_panda_then_host_di_and_host_di_then_panda(self):
    for panda_first in (True, False):
      with self.subTest(panda_first=panda_first):
        h = _Txn()
        h.cancel_and_wait_di(0)
        h.pull(399)
        self.assertEqual(h.tx()[0], SET_ACCEL)
        self.assertEqual(h.t.state, State.awaitingDiConfirmation)
        if panda_first:
          h.panda(1, True, True)
          self.assertEqual(h.t.state, State.awaitingDiConfirmation)
          self.assertFalse(h.t.enable_pending)
          h.di(True, 430)
        else:
          h.di(True, 430)
          self.assertEqual(h.t.state, State.awaitingDiConfirmation)
          self.assertFalse(h.t.enable_pending)
          h.panda(1, True, True)
        self.assertEqual(h.t.state, State.confirmed)
        self.assertTrue(h.t.enable_pending)

  def test_wrong_and_stale_panda_ack_fail_closed(self):
    h = _Txn()
    h.cancel_and_wait_di(0)
    h.pull(399)
    h.tx()
    h.di(True, 430)
    h.panda(0, True, True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    h.pull(500)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)
    h.stalk(IDLE, 510)
    self.assertEqual(h.t.state, State.idle)

    h = _Txn()
    h.panda(7, True, True)
    h.cancel_and_wait_di(0)
    h.pull(399)
    h.tx()
    h.di(True, 430)
    h.panda(7, True, True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

  def test_timeout_echo_duplicate_gap_reorder_wrap_cancel(self):
    h = _Txn()
    h.pull(0)
    h.tick(400)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

    h = _Txn()
    pair = h.cancel_and_wait_di(0)
    self.assertTrue(h.t.is_echo(CANCEL, pair[1], 10))
    h.t.sync_counter(pair[1])
    h.counter = (pair[1] + 1) & 0xF
    h.pull(399)
    self.assertEqual(h.t.state, State.reengageRequested)

    h = _Txn()
    h.stalk(IDLE, 0)
    h.stalk(MAIN)
    h.stalk(MAIN)
    self.assertEqual(h.t.state, State.cancelRequested)

    h = _Txn()
    h.pull(0)
    h.counter = (h.counter + 4) & 0xF
    h.stalk(IDLE, 20)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

    h = _Txn()
    h.pull(0)
    h.counter = (h.counter - 2) & 0xF
    h.stalk(IDLE, 20)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

    h = _Txn()
    h.counter = 15
    h.t.sync_counter(14)
    h.stalk(IDLE, 0)
    h.stalk(MAIN)
    self.assertEqual(h.t.state, State.cancelRequested)

    h = _Txn()
    h.pull(0)
    h.stalk(CANCEL, 10)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

  def test_direct_plus_minus_do_not_count_as_pulls_or_tx(self):
    h = _Txn()
    h.pull(0)
    before = h.t.state
    for lever in (CruiseButtons.RES_ACCEL, CruiseButtons.DECEL_SET, CruiseButtons.RES_ACCEL_2ND, CruiseButtons.DECEL_2ND):
      h.stalk(lever, 10)
    self.assertEqual(h.t.state, before)
    self.assertEqual(h.tx()[0], CANCEL)
    self.assertIsNone(h.tx())

  def test_blockers_brake_reconnect_and_recovery_need_idle(self):
    h = _Txn()
    h.pull(0)
    h.t.update_health(blocked=True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    h.t.update_health(blocked=False)
    h.pull(20)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)
    h.stalk(IDLE, 30)
    self.assertEqual(h.t.state, State.idle)

    h = _Txn()
    h.pull(0)
    h.t.update_health(blocked=False, brake_pressed=True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

    h = _Txn()
    h.t.bind_epoch(1, 0)
    h.pull(0)
    h.t.bind_epoch(2, 0)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

  def test_set_blocked_until_cancel_and_fresh_non_enabled_di(self):
    h = _Txn()
    h.pull(0)
    h.pull(50)
    self.assertEqual(h.t.state, State.cancelRequested)
    pair = h.tx()
    self.assertEqual(pair[0], CANCEL)
    h.di(True, 80)
    self.assertNotEqual(h.t.state, State.reengageRequested)
    self.assertIsNone(h.t.poll_tx(h.frame))
    h.di(False, 90)
    self.assertEqual(h.t.state, State.reengageRequested)

  def test_host_di_alone_does_not_confirm(self):
    h = _Txn()
    h.cancel_and_wait_di(0)
    h.pull(399)
    h.tx()
    h.di(True, 430)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    self.assertFalse(h.t.enable_pending)
    h.panda(1, True, False)
    self.assertFalse(h.t.enable_pending)
    h.panda(1, True, True)
    self.assertEqual(h.t.state, State.confirmed)

  def test_transaction_does_not_depend_on_engagement_mode(self):
    self.assertFalse(hasattr(StockCcTransaction, "mode"))
    self.assertNotIn("preapLateralEngagementMode", StockCcTransaction.__init__.__code__.co_varnames)

  def test_cancel_first_eligible_independent_of_frame_phase(self):
    for start in range(1, 10):
      with self.subTest(start=start):
        h = _Txn()
        h.pull(0)
        for frame in range(start, start + STOCK_CC_CANCEL_DELAY_FRAMES):
          self.assertIsNone(h.t.poll_tx(frame))
        first = start + STOCK_CC_CANCEL_DELAY_FRAMES
        self.assertEqual(h.t.poll_tx(first), CANCEL)
        self.assertNotEqual(first % STOCK_CC_TX_PERIOD_FRAMES, 0)

  def test_required_sources_fresh_uses_monotonic_observation_clock(self):
    seen = {key: 1_000_000_000 for key in REQUIRED_SOURCE_KEYS}
    self.assertTrue(required_sources_fresh(seen, 1_500_000_000))
    self.assertFalse(required_sources_fresh(seen, 2_000_000_001))
    frozen_parser = {key: 5 for key in REQUIRED_SOURCE_KEYS}
    self.assertTrue(required_sources_fresh(frozen_parser, 5))
    self.assertFalse(required_sources_fresh(frozen_parser, 5 + 1_000_000_001))
    missing = {key: None for key in REQUIRED_SOURCE_KEYS}
    self.assertFalse(required_sources_fresh(missing, 10))

  def test_cached_di_generation_cannot_confirm_cancel_or_set(self):
    h = _Txn()
    h.di(False, 0)
    h.pull(0)
    pair = h.tx()
    self.assertEqual(pair[0], CANCEL)
    bound = h.t._cancel_bound_generation
    h.t.update_di(False, 20, bound)
    self.assertFalse(h.t._post_cancel_di)
    self.assertEqual(h.t.state, State.awaitingCancelConfirmation)
    h.di(False, 30)
    self.assertTrue(h.t._post_cancel_di)
    self.assertEqual(h.t.state, State.awaitingSecondPull)
    h.pull(399)
    self.assertEqual(h.tx()[0], SET_ACCEL)
    set_bound = h.t._set_bound_generation
    h.t.update_di(True, 430, set_bound)
    h.panda(1, True, True)
    self.assertFalse(h.t.host_di_confirmed)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    h.di(True, 440)
    self.assertTrue(h.t.host_di_confirmed)
    self.assertEqual(h.t.state, State.confirmed)

  def test_di_generation_wrap_is_strictly_newer(self):
    h = _Txn()
    h.di(False, 0, generation=0xFFFFFFFF)
    h.pull(0)
    self.assertEqual(h.tx()[0], CANCEL)
    self.assertEqual(h.t._cancel_bound_generation, 0xFFFFFFFF)
    h.t.update_di(False, 20, 0xFFFFFFFF)
    self.assertFalse(h.t._post_cancel_di)
    h.di(False, 30, generation=0)
    self.assertTrue(h.t._post_cancel_di)

  def test_in_flight_timeout_progresses_during_total_silence(self):
    h = _Txn()
    h.pull(0)
    self.assertEqual(h.t.state, State.cancelRequested)
    h.tick(now=299)
    self.assertEqual(h.t.state, State.cancelRequested)
    h.tick(now=300)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

  def _confirm(self):
    h = _Txn()
    h.cancel_and_wait_di(0)
    h.pull(399)
    self.assertEqual(h.tx()[0], SET_ACCEL)
    h.di(True, 430)
    h.panda(1, True, True)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t.enable_pending)
    self.assertTrue(h.t.host_di_confirmed)
    return h

  def test_brake_revokes_confirmed_authority_and_clears_pending(self):
    h = self._confirm()
    h.t.update_health(blocked=False, brake_pressed=True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)

  def test_required_source_blocker_revokes_confirmed_authority(self):
    h = self._confirm()
    h.t.update_health(blocked=True, brake_pressed=False)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)


  def test_confirmed_panda_authority_is_continuously_required(self):
    h = self._confirm()
    h.panda(1, True, True)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t.enable_pending)
    self.assertTrue(h.t.host_di_confirmed)

    h = self._confirm()
    h.panda(1, False, True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)

    h = self._confirm()
    h.panda(1, True, False)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)

    h = self._confirm()
    h.panda(2, True, True)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)

  def test_missing_panda_fails_active_and_leaves_idle(self):
    h = self._confirm()
    h.panda(None, False, False)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)

    idle = _Txn()
    idle.panda(None, False, False)
    self.assertEqual(idle.t.state, State.idle)

    h = _Txn()
    h.cancel_and_wait_di(0)
    h.pull(399)
    self.assertEqual(h.tx()[0], SET_ACCEL)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    h.panda(None, False, False)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)

  def test_confirmed_c2_c3_batch_keeps_terminal_until_publish(self):
    h = self._confirm()
    counter = h.t._stalk_counter
    h.t.update_stalk(IDLE, (counter + 2) & 0xF, 500)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    h.t.update_stalk(IDLE, (counter + 3) & 0xF, 500)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertTrue(h.t._need_release)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)
    h.t.update_stalk(IDLE, (counter + 4) & 0xF, 510)
    self.assertEqual(h.t.state, State.idle)



  def test_confirmed_di_fall_fails_closed(self):
    h = self._confirm()
    h.di(False)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)
    sp = h.publish()
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    self.assertFalse(sp.preapStockCcEnablePending)
    self.assertFalse(sp.preapStockCcHostDiConfirmed)

  def test_confirmed_di_repeat_and_same_generation_are_not_terminal(self):
    h = self._confirm()
    gen = h.t._di_generation
    self.assertTrue(h.t._di_enabled)
    h.t.update_di(False, h.now, gen)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t._di_enabled)
    h.t.update_di(False, h.now, gen)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t._di_enabled)
    self.assertTrue(h.t.enable_pending)

  def test_stale_false_then_fresh_false_revokes_confirmed_authority(self):
    h = self._confirm()
    gen = h.t._di_generation
    self.assertTrue(h.t._di_enabled)
    h.t.update_di(False, h.now, gen)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t._di_enabled)
    self.assertTrue(h.t.enable_pending)
    self.assertEqual(h.t._di_generation, gen)
    h.t.update_di(False, h.now, (gen + 1) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t.enable_pending)
    self.assertFalse(h.t.host_di_confirmed)

  def test_confirmed_di_fresh_repeat_remains_confirmed(self):
    h = self._confirm()
    gen = h.t._di_generation
    self.assertTrue(h.t._di_enabled)
    h.t.update_di(True, h.now, (gen + 1) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t._di_enabled)
    self.assertTrue(h.t.enable_pending)
    self.assertTrue(h.t.host_di_confirmed)
    self.assertEqual(h.t._di_generation, (gen + 1) & 0xFFFFFFFF)
    h.t.update_di(True, h.now, (gen + 2) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertTrue(h.t._di_enabled)
    self.assertTrue(h.t.enable_pending)
    self.assertTrue(h.t.host_di_confirmed)
    self.assertEqual(h.t._di_generation, (gen + 2) & 0xFFFFFFFF)

  def test_pre_confirmation_di_disengaged_is_not_confirmed_fall(self):
    h = _Txn()
    h.cancel_and_wait_di(0)
    h.pull(399)
    self.assertEqual(h.tx()[0], SET_ACCEL)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    h.di(False)
    self.assertEqual(h.t.state, State.awaitingDiConfirmation)
    self.assertFalse(h.t.host_di_confirmed)

  def test_post_cancel_di_499_allows_500_501_fail(self):
    for elapsed, allowed in ((499, True), (500, False), (501, False)):
      with self.subTest(elapsed=elapsed):
        h = _Txn()
        h.pull(0)
        pair = h.tx()
        self.assertEqual(pair[0], CANCEL)
        self.assertEqual(h.t.state, State.awaitingCancelConfirmation)
        h.di(False, now=elapsed)
        if allowed:
          self.assertEqual(h.t.state, State.awaitingSecondPull)
          self.assertTrue(h.t._post_cancel_di)
        else:
          self.assertEqual(h.t.state, State.cancelledOrFailed)
          self.assertFalse(h.t._post_cancel_di)
          self.assertFalse(h.t.enable_pending)

  def test_post_cancel_di_before_tick_keeps_handshake(self):
    h = _Txn()
    h.pull(0)
    self.assertEqual(h.tx()[0], CANCEL)
    h.di(False, now=499)
    self.assertEqual(h.t.state, State.awaitingSecondPull)
    h.tick(500)
    self.assertEqual(h.t.state, State.awaitingSecondPull)

  def test_tick_before_post_cancel_di_fails_and_cannot_resurrect(self):
    h = _Txn()
    h.pull(0)
    self.assertEqual(h.tx()[0], CANCEL)
    h.tick(500)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    h.di(False, now=500)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t._post_cancel_di)

  def test_post_cancel_di_deadline_uint32_wrap(self):
    origin = 0xFFFFFF00
    h = _Txn()
    h.pull(origin)
    self.assertEqual(h.tx()[0], CANCEL)
    h.di(False, now=(origin + 499) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.awaitingSecondPull)
    self.assertTrue(h.t._post_cancel_di)

    h = _Txn()
    h.pull(origin)
    self.assertEqual(h.tx()[0], CANCEL)
    h.di(False, now=(origin + 500) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    self.assertFalse(h.t._post_cancel_di)

    h = _Txn()
    h.pull(origin)
    self.assertEqual(h.tx()[0], CANCEL)
    h.di(False, now=(origin + 501) & 0xFFFFFFFF)
    self.assertEqual(h.t.state, State.cancelledOrFailed)

  def test_direct_adjustment_levers_preserve_pending_origin_through_set(self):
    for lever in PASSTHROUGH_LEVERS:
      with self.subTest(lever=lever):
        h = _Txn()
        h.pull(0)
        h.stalk(lever, 10)
        h.stalk(IDLE, 11)
        pair = h.tx()
        self.assertEqual(pair[0], CANCEL)
        self.assertNotEqual(pair[0], MAIN)
        h.di(False, 30)
        h.pull(399)
        self.assertEqual(h.t.state, State.reengageRequested)
        set_pair = h.tx()
        self.assertEqual(set_pair[0], SET_ACCEL)
        self.assertNotEqual(set_pair[0], MAIN)
        self.assertIsNone(h.t.poll_tx(h.frame))

  def test_every_non_none_poll_tx_is_cancel_or_set_never_main(self):
    observed = []

    def drain(h, frames=80):
      for _ in range(frames):
        lever = h.t.poll_tx(h.frame)
        h.frame += 1
        if lever is not None:
          observed.append(lever)
          self.assertIn(lever, (CANCEL, SET_ACCEL))
          self.assertNotEqual(lever, MAIN)
          counter = h.t.tx_counter()
          h.t.note_tx(lever, counter, h.now)
          h.counter = (counter + 1) & 0xF
          return lever
      return None

    h = _Txn(active=False)
    h.pull(0)
    self.assertIsNone(drain(h, 20))

    h = _Txn()
    self.assertIsNone(drain(h, 20))
    h.pull(0)
    self.assertEqual(drain(h), CANCEL)
    self.assertIsNone(h.t.poll_tx(h.frame))
    h.di(False, 30)
    self.assertIsNone(drain(h, 20))
    h.pull(399)
    self.assertEqual(drain(h), SET_ACCEL)
    self.assertIsNone(h.t.poll_tx(h.frame))
    h.di(True, 430)
    h.panda(1, True, True)
    self.assertEqual(h.t.state, State.confirmed)
    self.assertIsNone(drain(h, 20))

    h = _Txn()
    h.pull(0)
    h.t.update_health(blocked=True)
    self.assertIsNone(drain(h, 20))

    h = _Txn()
    h.pull(0)
    for lever in PASSTHROUGH_LEVERS:
      h.stalk(lever, 10)
    self.assertEqual(drain(h), CANCEL)
    self.assertIsNone(h.t.poll_tx(h.frame))
    self.assertTrue(observed)
    self.assertNotIn(MAIN, observed)
    self.assertTrue(all(lever in (CANCEL, SET_ACCEL) for lever in observed))

  def test_neutralized_terminal_projection_does_not_consume_latch(self):
    h = _Txn()
    h.pull(0)
    h.t.update_health(blocked=True)
    sp = structs.CarStateSP()
    h.t.publish(sp)
    self.assertEqual(sp.preapStockCcState, State.cancelledOrFailed)
    sp.preapStockCcState = State.idle
    h.t.acknowledge_publication(sp)
    h.stalk(IDLE, 20)
    self.assertEqual(h.t.state, State.cancelledOrFailed)
    published = h.publish()
    self.assertEqual(published.preapStockCcState, State.cancelledOrFailed)
    h.stalk(IDLE, 30)
    self.assertEqual(h.t.state, State.idle)


if __name__ == "__main__":
  unittest.main()
