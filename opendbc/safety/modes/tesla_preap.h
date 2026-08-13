#pragma once

#include "opendbc/safety/declarations.h"

#define PREAP_MODE_MASK 0x3U
#define PREAP_MODE_INDEPENDENT 0U
#define PREAP_MODE_CRUISE_COUPLED 1U
#define PREAP_MODE_LONGITUDINAL_ONLY 2U
#define PREAP_MODE_INVALID 3U

#define PREAP_FLAG_ENABLE_PEDAL (1U << 2)
#define PREAP_FLAG_RADAR_EMULATION (1U << 3)
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE (1U << 4)

#define PREAP_STALK_DOUBLE_PULL_US 400000U
#define PREAP_STOCK_CC_CONFIRM_US 500000U
#define PREAP_REQUIRED_SOURCE_MAX_AGE_US 1000000U
#define PREAP_HANDS_ON_RESUME_US 1000000U
#define PREAP_PEDAL_GAS_THRESHOLD 650

static bool preap_enable_pedal = false;
static uint8_t preap_mode = PREAP_MODE_INVALID;

static bool preap_gear_seen = false;
static bool preap_gear_drive = false;
static uint32_t preap_gear_ts = 0U;
static bool preap_doors_seen = false;
static bool preap_doors_closed = false;
static uint32_t preap_doors_ts = 0U;
static bool preap_epas_seen = false;
static bool preap_epas_healthy = false;
static uint32_t preap_epas_ts = 0U;
static bool preap_di_brake_seen = false;
static bool preap_di_brake_pressed = false;
static uint32_t preap_di_brake_ts = 0U;
static bool preap_brake_message_seen = false;
static bool preap_brake_message_pressed = false;
static uint32_t preap_brake_message_ts = 0U;
static bool preap_gas_seen = false;
static uint32_t preap_gas_ts = 0U;

static bool preap_stalk_armed = false;
static bool preap_pull_pending = false;
static uint32_t preap_first_pull_ts = 0U;
static bool preap_brake_paused_lateral = false;
static bool preap_stock_cc_reengage_authorized = false;
static bool preap_stock_cc_reengage_sent = false;
static bool preap_stock_cc_cancel_authorized = false;
static uint32_t preap_stock_cc_deadline_ts = 0U;
static bool preap_hands_on_clear_timing = false;
static uint32_t preap_hands_on_clear_ts = 0U;
static bool preap_stalk_counter_seen = false;
static uint8_t preap_stalk_counter_last = 0U;


static bool tesla_preap_source_fresh(bool seen, uint32_t timestamp, uint32_t now) {
  return seen && (safety_get_ts_elapsed(now, timestamp) <= PREAP_REQUIRED_SOURCE_MAX_AGE_US);
}

static void tesla_preap_clear_pull_state(void) {
  preap_stalk_armed = false;
  preap_pull_pending = false;
  preap_first_pull_ts = 0U;
  preap_stock_cc_reengage_authorized = false;
  preap_stock_cc_reengage_sent = false;
  preap_stock_cc_cancel_authorized = false;
  preap_stock_cc_deadline_ts = 0U;
}

static void tesla_preap_clear_stock_cc_confirmation(void) {
  stock_cc_reengage_confirmed = false;
  preap_stock_cc_reengage_authorized = false;
  preap_stock_cc_reengage_sent = false;
  preap_stock_cc_deadline_ts = 0U;
}

static void tesla_preap_exit(DisengageReason reason) {
  controls_allowed = false;
  mads_exit_controls(reason);
  controls_allowed_lateral = false;
  m_mads_state.controls_requested_lateral = false;
  tesla_preap_clear_pull_state();
  tesla_preap_clear_stock_cc_confirmation();
  preap_brake_paused_lateral = false;
}

static bool tesla_preap_required_sources_valid(uint32_t now) {
  const bool sources_fresh = tesla_preap_source_fresh(preap_gear_seen, preap_gear_ts, now) &&
                             tesla_preap_source_fresh(preap_doors_seen, preap_doors_ts, now) &&
                             tesla_preap_source_fresh(preap_epas_seen, preap_epas_ts, now) &&
                             tesla_preap_source_fresh(preap_di_brake_seen, preap_di_brake_ts, now) &&
                             tesla_preap_source_fresh(preap_brake_message_seen, preap_brake_message_ts, now);
  return (preap_mode != PREAP_MODE_INVALID) && sources_fresh && preap_gear_drive && preap_doors_closed &&
         preap_epas_healthy;
}

static bool tesla_preap_required_sources_ready(uint32_t now) {
  return tesla_preap_required_sources_valid(now) && !preap_di_brake_pressed && !preap_brake_message_pressed;
}

static void tesla_preap_request_lateral(void) {
  if ((preap_mode != PREAP_MODE_LONGITUDINAL_ONLY) && m_mads_state.system_enabled) {
    m_mads_state.controls_requested_lateral = false;
    controls_allowed_lateral = true;
    m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
    m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

static uint8_t tesla_preap_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;
  if (msg->addr == 0x370U) {
    counter = msg->data[6] & 0xFU;
  } else if (msg->addr == 0x108U) {
    counter = msg->data[1] >> 5;
  } else if (msg->addr == 0x118U) {
    counter = msg->data[4] & 0xFU;
  } else if (msg->addr == 0x368U) {
    counter = msg->data[5] >> 4;
  } else if (msg->addr == 0x155U) {
    counter = (msg->data[7] >> 3) & 0xFU;
  } else if (msg->addr == 0x45U) {
    counter = msg->data[6] >> 4;
  } else if (msg->addr == 0x552U) {
    counter = msg->data[4] & 0xFU;
  } else {
  }
  return counter;
}


static int tesla_preap_checksum_byte(uint32_t addr) {
  int checksum_byte = -1;
  if ((addr == 0x370U) || (addr == 0x108U) || (addr == 0x368U) || (addr == 0x45U) || (addr == 0x3E9U)) {
    checksum_byte = 7;
  } else if ((addr == 0x118U) || (addr == 0x551U) || (addr == 0x552U)) {
    checksum_byte = 5;
  } else if (addr == 0x155U) {
    checksum_byte = 4;
  } else if (addr == 0x488U) {
    checksum_byte = 3;
  } else {
  }
  return checksum_byte;
}

static uint32_t tesla_preap_get_checksum(const CANPacket_t *msg) {
  const int checksum_byte = tesla_preap_checksum_byte(msg->addr);
  return (checksum_byte >= 0) ? msg->data[checksum_byte] : 0U;
}

static uint8_t tesla_preap_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0xFFU;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1) ^ 0x1DU) : (uint8_t)(crc << 1);
    }
  }
  return crc ^ 0xFFU;
}

static uint32_t tesla_preap_compute_checksum(const CANPacket_t *msg) {
  const int checksum_byte = tesla_preap_checksum_byte(msg->addr);
  uint8_t checksum = 0U;
  if (msg->addr == 0x45U) {
    checksum = tesla_preap_crc8(msg->data, 7);
  } else if (checksum_byte >= 0) {
    checksum = (uint8_t)((msg->addr & 0xFFU) + ((msg->addr >> 8) & 0xFFU));
    const int msg_len = (int)GET_LEN(msg);
    for (int i = 0; i < msg_len; i++) {
      if (i != checksum_byte) {
        checksum += msg->data[i];
      }
    }
  } else {
  }
  return checksum;
}

static bool tesla_preap_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = true;
  if (msg->addr == 0x155U) {
    const uint8_t quality = msg->data[7] & 0x3U;
    valid = (quality == 1U) || (quality == 2U);
  }
  return valid;
}

static void tesla_preap_process_first_pull(uint32_t now) {
  if (controls_allowed) {
    controls_allowed = false;
    tesla_preap_clear_stock_cc_confirmation();
    if (!preap_enable_pedal) {
      preap_stock_cc_cancel_authorized = true;
    }
    if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BUTTON);
    }
  } else if (preap_mode == PREAP_MODE_INDEPENDENT) {
    tesla_preap_request_lateral();
  } else {
  }
  preap_pull_pending = true;
  preap_first_pull_ts = now;
}

static void tesla_preap_process_second_pull(uint32_t now) {
  preap_pull_pending = false;
  const bool engagement_ready = tesla_preap_required_sources_ready(now) && !gas_pressed && preap_gas_seen &&
                                tesla_preap_source_fresh(preap_gas_seen, preap_gas_ts, now);
  if (engagement_ready) {
    if (preap_enable_pedal) {
      controls_allowed = true;
      if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
        tesla_preap_request_lateral();
      }
    } else {
      preap_stock_cc_reengage_authorized = true;
      preap_stock_cc_deadline_ts = now;
      stock_cc_reengage_confirmed = false;
    }
  } else {
    tesla_preap_clear_pull_state();
  }
}

static void tesla_preap_process_main_pull(uint32_t now) {
  const uint32_t elapsed = safety_get_ts_elapsed(now, preap_first_pull_ts);
  if (preap_pull_pending && (elapsed > 0U) && (elapsed < PREAP_STALK_DOUBLE_PULL_US)) {
    tesla_preap_process_second_pull(now);
  } else {
    tesla_preap_process_first_pull(now);
  }
}

static void tesla_preap_apply_brake_policy(bool brake_rising, bool brake_released) {
  if (brake_rising) {
    controls_allowed = false;
    tesla_preap_clear_stock_cc_confirmation();
    preap_pull_pending = false;
    preap_stalk_armed = false;

    if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    } else if ((preap_mode == PREAP_MODE_INDEPENDENT) && m_mads_state.disengage_lateral_on_brake) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    } else if ((preap_mode == PREAP_MODE_INDEPENDENT) && m_mads_state.pause_lateral_on_brake && controls_allowed_lateral) {
      controls_allowed_lateral = false;
      preap_brake_paused_lateral = true;
    } else {
    }
  }

  if (brake_released && preap_brake_paused_lateral && tesla_preap_required_sources_ready(microsecond_timer_get())) {
    preap_brake_paused_lateral = false;
    tesla_preap_request_lateral();
  }
}

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  if (msg->returned == 0U) {
    const bool is_pedal_sensor = msg->addr == 0x552U;
    if (is_pedal_sensor) {
      const int pedal_raw = (msg->data[0] << 8) | msg->data[1];
      const uint8_t pedal_state = msg->data[4] >> 4;
      preap_gas_seen = pedal_state == 0U;
      preap_gas_ts = now;
      gas_pressed = !preap_gas_seen || (pedal_raw > PREAP_PEDAL_GAS_THRESHOLD);
    }
    if (!is_pedal_sensor && (msg->bus == 0U)) {
  if (msg->addr == 0x370U) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    const uint8_t hands_on_level = msg->data[4] >> 6;
    const uint8_t eac_status = msg->data[6] >> 5;
    const uint8_t eac_error_code = msg->data[2] >> 4;
    const bool epas_fault = (eac_status == 0U) && (eac_error_code >= 6U) && (eac_error_code <= 9U);

    update_sample(&angle_meas, angle_meas_new);
    preap_epas_seen = true;
    preap_epas_healthy = !epas_fault;
    preap_epas_ts = now;

    if (epas_fault) {
      steering_control_inhibited = false;
      preap_hands_on_clear_timing = false;
      tesla_preap_exit(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    } else if (hands_on_level >= 2U) {
      steering_control_inhibited = true;
      preap_hands_on_clear_timing = false;
    } else if (steering_control_inhibited) {
      if (!preap_hands_on_clear_timing) {
        preap_hands_on_clear_timing = true;
        preap_hands_on_clear_ts = now;
      } else if (safety_get_ts_elapsed(now, preap_hands_on_clear_ts) >= PREAP_HANDS_ON_RESUME_US) {
        steering_control_inhibited = false;
        preap_hands_on_clear_timing = false;
      } else {
      }
    } else {
      preap_hands_on_clear_timing = false;
    }
  }

  if (msg->addr == 0x155U) {
    const float speed = (((msg->data[5] << 8) | msg->data[6]) * 0.01F) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > (0.5F * KPH_TO_MS);
  }

  if (msg->addr == 0x108U) {
    preap_gas_seen = true;
    preap_gas_ts = now;
    if (!preap_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  if (msg->addr == 0x20AU) {
    const bool brake_was_pressed = preap_brake_message_pressed || preap_di_brake_pressed;
    const uint8_t brake_status = (msg->data[0] >> 2) & 0x3U;
    preap_brake_message_seen = (brake_status == 1U) || (brake_status == 2U);
    preap_brake_message_pressed = brake_status == 2U;
    preap_brake_message_ts = now;
    const bool brake_now = preap_brake_message_pressed || preap_di_brake_pressed;
    brake_pressed = brake_now;
    tesla_preap_apply_brake_policy(!brake_was_pressed && brake_now, brake_was_pressed && !brake_now);
    if (!preap_brake_message_seen) {
      tesla_preap_exit(MADS_DISENGAGE_REASON_BRAKE);
    }
  }

  if (msg->addr == 0x118U) {
    const bool brake_was_pressed = preap_brake_message_pressed || preap_di_brake_pressed;
    const uint8_t gear = (msg->data[1] >> 4) & 0x7U;
    const uint8_t brake_state = (msg->data[4] >> 4) & 0x3U;
    preap_gear_seen = true;
    preap_gear_drive = gear == 4U;
    preap_gear_ts = now;
    preap_di_brake_seen = brake_state <= 1U;
    preap_di_brake_pressed = ((msg->data[1] & 0x80U) != 0U) || (brake_state == 1U);
    preap_di_brake_ts = now;
    const bool brake_now = preap_brake_message_pressed || preap_di_brake_pressed;
    brake_pressed = brake_now;
    tesla_preap_apply_brake_policy(!brake_was_pressed && brake_now, brake_was_pressed && !brake_now);
    if (!preap_gear_drive || !preap_di_brake_seen) {
      tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
    }
  }

  if (msg->addr == 0x318U) {
    const uint8_t door_fl = (msg->data[1] >> 4) & 0x3U;
    const uint8_t door_fr = (msg->data[1] >> 6) & 0x3U;
    const uint8_t door_rl = (msg->data[2] >> 6) & 0x3U;
    const uint8_t door_rr = (msg->data[3] >> 5) & 0x3U;
    const uint8_t front_trunk = (msg->data[6] >> 2) & 0x3U;
    const uint8_t boot_state = (msg->data[5] >> 6) & 0x3U;
    preap_doors_seen = true;
    preap_doors_closed = (door_fl == 0U) && (door_fr == 0U) && (door_rl == 0U) && (door_rr == 0U) &&
                         (front_trunk == 0U) && (boot_state == 0U);
    preap_doors_ts = now;
    if (!preap_doors_closed) {
      tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
    }
  }

  if (msg->addr == 0x368U) {
    const uint8_t cruise_state = (msg->data[1] >> 4) & 0x7U;
    const bool cruise_engaged = (cruise_state == 2U) || (cruise_state == 3U) || (cruise_state == 4U);
    if (!preap_enable_pedal && preap_stock_cc_reengage_sent) {
      if ((safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts) < PREAP_STOCK_CC_CONFIRM_US) && cruise_engaged &&
          tesla_preap_required_sources_ready(now)) {
        controls_allowed = true;
        stock_cc_reengage_confirmed = true;
        preap_stock_cc_reengage_sent = false;
        if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
          tesla_preap_request_lateral();
        }
      } else if (safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts) >= PREAP_STOCK_CC_CONFIRM_US) {
        tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
      } else {
      }
    }
  }

  if (msg->addr == 0x45U) {
    const uint8_t lever = msg->data[0] & 0x3FU;
    const uint8_t counter = tesla_preap_get_counter(msg);
    const bool counter_consecutive = !preap_stalk_counter_seen || (counter == ((preap_stalk_counter_last + 1U) & 0xFU));
    preap_stalk_counter_seen = true;
    preap_stalk_counter_last = counter;

    if (lever == 1U) {
      tesla_preap_exit(MADS_DISENGAGE_REASON_BUTTON);
    } else if (!counter_consecutive) {
      preap_stalk_armed = false;
      preap_pull_pending = false;
    } else if (lever == 0U) {
      if (tesla_preap_required_sources_ready(now)) {
        preap_stalk_armed = true;
      }
    } else if (lever == 2U) {
      if (preap_stalk_armed && tesla_preap_required_sources_ready(now)) {
        preap_stalk_armed = false;
        tesla_preap_process_main_pull(now);
      }
    } else {
      // Unknown stalk positions cannot carry engagement authority.
      preap_stalk_armed = false;
      preap_pull_pending = false;
    }
  }

  if (!tesla_preap_required_sources_valid(now) &&
      (controls_allowed || controls_allowed_lateral || preap_pull_pending || preap_stock_cc_reengage_sent)) {
    preap_hands_on_clear_timing = false;
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
    }
  }
}

static void tesla_preap_invalid_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  preap_hands_on_clear_timing = false;
  tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
}

static void tesla_preap_tick(bool rx_checks_invalid) {
  const uint32_t now = microsecond_timer_get();
  if (rx_checks_invalid || !tesla_preap_required_sources_valid(now)) {
    preap_hands_on_clear_timing = false;
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
  if ((preap_stock_cc_reengage_authorized || preap_stock_cc_reengage_sent) &&
      (safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts) >= PREAP_STOCK_CC_CONFIRM_US)) {
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
}

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return false;
}

static bool tesla_preap_fwd_hook(int bus_num, int addr) {
  SAFETY_UNUSED(bus_num);
  SAFETY_UNUSED(addr);
  return true;
}

static safety_config tesla_preap_init(uint16_t param) {
  static RxCheck preap_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .max_counter = 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x20A, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x45, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
  };
  static RxCheck preap_rx_checks_with_pedal[] = {
    {.msg = {{0x370, 0, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .max_counter = 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x20A, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x45, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    {.msg = {{0x552, 0, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true},
             {0x552, 2, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }}},
  };

  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL);
  preap_mode = (uint8_t)(current_safety_param_sp & PREAP_MODE_MASK);

  preap_gear_seen = false;
  preap_gear_drive = false;
  preap_gear_ts = 0U;
  preap_doors_seen = false;
  preap_doors_closed = false;
  preap_doors_ts = 0U;
  preap_epas_seen = false;
  preap_epas_healthy = false;
  preap_epas_ts = 0U;
  preap_di_brake_seen = false;
  preap_di_brake_pressed = false;
  preap_di_brake_ts = 0U;
  preap_brake_message_seen = false;
  preap_brake_message_pressed = false;
  preap_brake_message_ts = 0U;
  preap_gas_seen = false;
  preap_gas_ts = 0U;
  preap_brake_paused_lateral = false;
  preap_hands_on_clear_timing = false;
  preap_hands_on_clear_ts = 0U;
  steering_control_inhibited = false;
  stock_cc_reengage_counter = 0U;
  stock_cc_reengage_confirmed = false;
  preap_stalk_counter_seen = false;
  preap_stalk_counter_last = 0U;
  tesla_preap_clear_pull_state();

  safety_config ret = {
    .rx_checks = preap_rx_checks,
    .rx_checks_len = 8,
    .tx_msgs = NULL,
    .tx_msgs_len = 0,
    .disable_forwarding = true,
  };
  if (preap_enable_pedal) {
    ret.rx_checks = preap_rx_checks_with_pedal;
    ret.rx_checks_len = 9;
  }
  return ret;
}

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .invalid_rx = tesla_preap_invalid_rx_hook,
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .tick = tesla_preap_tick,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_counter = tesla_preap_get_counter,
  .get_quality_flag_valid = tesla_preap_get_quality_flag_valid,
};
