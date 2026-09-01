#pragma once

#include "opendbc/safety/declarations.h"

#define PREAP_MODE_MASK 0x3U
#define PREAP_MODE_INDEPENDENT 0U
#define PREAP_MODE_CRUISE_COUPLED 1U
#define PREAP_MODE_LONGITUDINAL_ONLY 2U
#define PREAP_MODE_INVALID 3U

#define PREAP_FLAG_ENABLE_PEDAL (1U << 2)
#define PREAP_FLAG_RADAR_EMULATION (1U << 3)
// Leftover bit. Position comes from host 0x560, not this flag.
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE (1U << 4)
#define PREAP_FLAG_PEDAL_BUS_ZERO (1U << 5)
#define PREAP_FLAG_PEDAL_CALIBRATION (1U << 6)
#define PREAP_GEAR_NEUTRAL 3U

#define PREAP_STALK_DOUBLE_PULL_US 400000U
#define PREAP_STOCK_CC_CANCEL_US 100000U
#define PREAP_STOCK_CC_OBSERVATION_US 10000U
#define PREAP_STOCK_CC_DELIVERY_US 10000U
#define PREAP_STOCK_CC_CANCEL_AUTH_US (PREAP_STOCK_CC_CANCEL_US + PREAP_STOCK_CC_OBSERVATION_US + PREAP_STOCK_CC_DELIVERY_US)
#define PREAP_STOCK_CC_CONFIRM_US 500000U
#define PREAP_REQUIRED_SOURCE_MAX_AGE_US 1000000U
#define PREAP_HANDS_ON_RESUME_US 1000000U
#define PREAP_PEDAL_GAS_THRESHOLD 650
#define PREAP_PEDAL_FEEDBACK_TIMEOUT_US 500000U
#define PREAP_CANCEL_ECHO_US 600000U
#define PREAP_SPOOF_ECHO_US 300000U
#define PREAP_STALK_RES_ACCEL_2ND 4U
#define PREAP_STALK_DECEL_2ND 8U
#define PREAP_STALK_RES_ACCEL 16U
#define PREAP_STALK_DECEL_SET 32U

#define PREAP_GET_BYTES_04(msg) GET_BYTES((msg), 0, 4)
#define PREAP_GET_BYTES_48(msg) GET_BYTES((msg), 4, 4)

#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

static bool preap_enable_pedal = false;
static bool preap_pedal_calibration = false;
static uint8_t preap_pedal_bus = 2U;
static bool preap_pedal_feedback_counter_seen = false;
static uint8_t preap_pedal_feedback_counter = 0U;
static uint32_t preap_pedal_feedback_advance_ts = 0U;
static bool preap_pedal_feedback_healthy = false;
static bool preap_pedal_tx_counter_seen = false;
static uint8_t preap_pedal_tx_counter = 0U;
static uint8_t preap_mode = PREAP_MODE_INVALID;

static bool preap_gear_seen = false;
static bool preap_gear_drive = false;
static bool preap_gear_neutral = false;
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
static uint32_t preap_stock_cc_deadline_ts = 0U;
static bool preap_stock_cc_cancel_authorized = false;
static bool preap_stock_cc_cancel_sent = false;
static uint32_t preap_stock_cc_cancel_sent_ts = 0U;
static bool preap_stock_cc_post_cancel_di = false;
static bool preap_stock_cc_pull2_latched = false;
static uint32_t preap_stock_cc_pull2_ts = 0U;
static bool preap_stock_cc_awaiting_di_rise = false;
static uint8_t preap_stock_cc_expected_counter = 0U;
static bool preap_stock_cc_di_engaged = false;
static uint32_t preap_stock_cc_di_ts = 0U;
static bool preap_stock_cc_di_seen = false;
static bool preap_stock_cc_di_prior_engaged = false;
static bool preap_stock_cc_di_prior_valid = false;
static uint8_t preap_live_stw[8] = {0};
static bool preap_live_stw_valid = false;
static bool preap_echo_active = false;
static uint8_t preap_echo_lever = 0U;
static uint8_t preap_echo_counter = 0U;
static uint32_t preap_echo_ts = 0U;
static uint32_t preap_echo_window_us = 0U;
static bool preap_hands_on_clear_timing = false;
static uint32_t preap_hands_on_clear_ts = 0U;
static bool preap_stalk_counter_seen = false;
static uint8_t preap_stalk_counter_last = 0U;

static bool preap_radar_emulation = false;
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static int preap_radar_epas_type = 0;
static int preap_radar_position = 0;
static uint8_t preap_radar_vin[17];
static uint8_t preap_radar_vin_complete = 0;

// Host→panda donor config. 0x560 never goes on the car; tesla_preap_tx_hook
// consumes it. Layout matches Tinkla 0.6.6 create_radar_VIN_msg.
#define PREAP_RADAR_VIN_ADDR 0x560U
#define PREAP_RADAR_UDS_ADDR 0x641U

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
#define PREAP_RADAR_GTW_CAPTURE_MAX 16
static bool preap_radar_car_config_captured = false;
static CANPacket_t preap_radar_car_config_capture;
static bool preap_radar_vin_feed_captured = false;
static CANPacket_t preap_radar_vin_feed_capture;
static int preap_radar_gtw_count = 0;
static CANPacket_t preap_radar_gtw_capture[PREAP_RADAR_GTW_CAPTURE_MAX];
#endif

static bool tesla_preap_source_fresh(bool seen, uint32_t timestamp, uint32_t now) {
  const uint32_t elapsed = safety_get_ts_elapsed(now, timestamp);
  return seen && (elapsed <= PREAP_REQUIRED_SOURCE_MAX_AGE_US);
}

static void tesla_preap_clear_stock_cc_tx_state(void) {
  preap_stock_cc_reengage_authorized = false;
  preap_stock_cc_reengage_sent = false;
  preap_stock_cc_deadline_ts = 0U;
  preap_stock_cc_cancel_authorized = false;
  preap_stock_cc_cancel_sent = false;
  preap_stock_cc_cancel_sent_ts = 0U;
  preap_stock_cc_post_cancel_di = false;
  preap_stock_cc_pull2_latched = false;
  preap_stock_cc_pull2_ts = 0U;
  preap_stock_cc_awaiting_di_rise = false;
  preap_stock_cc_expected_counter = 0U;
  preap_echo_active = false;
}

static void tesla_preap_retire_confirmed_stock_cc_handshake(void) {
  preap_stock_cc_reengage_authorized = false;
  preap_stock_cc_reengage_sent = false;
  preap_stock_cc_deadline_ts = 0U;
  preap_stock_cc_cancel_authorized = false;
  preap_stock_cc_cancel_sent = false;
  preap_stock_cc_cancel_sent_ts = 0U;
  preap_stock_cc_post_cancel_di = false;
  preap_stock_cc_pull2_latched = false;
  preap_stock_cc_pull2_ts = 0U;
  preap_stock_cc_awaiting_di_rise = false;
  preap_stock_cc_expected_counter = 0U;
  // Keep preap_echo_active: the just-sent SET may still return on the bus.
}

static bool tesla_preap_cancel_window_open(uint32_t now) {
  const uint32_t elapsed = safety_get_ts_elapsed(now, preap_first_pull_ts);
  return preap_stock_cc_cancel_authorized && !preap_stock_cc_cancel_sent &&
         (elapsed <= PREAP_STOCK_CC_CANCEL_AUTH_US);
}

static void tesla_preap_expire_unsent_cancel(uint32_t now) {
  if (preap_stock_cc_cancel_authorized && !preap_stock_cc_cancel_sent &&
      (safety_get_ts_elapsed(now, preap_first_pull_ts) > PREAP_STOCK_CC_CANCEL_AUTH_US)) {
    tesla_preap_clear_stock_cc_tx_state();
  }
}

static void tesla_preap_clear_pull_state(void) {
  preap_stalk_armed = false;
  preap_pull_pending = false;
  preap_first_pull_ts = 0U;
  tesla_preap_clear_stock_cc_tx_state();
}

static void tesla_preap_clear_stock_cc_confirmation(void) {
  stock_cc_reengage_confirmed = false;
  tesla_preap_clear_stock_cc_tx_state();
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
  const bool gear_fresh = tesla_preap_source_fresh(preap_gear_seen, preap_gear_ts, now);
  const bool doors_fresh = tesla_preap_source_fresh(preap_doors_seen, preap_doors_ts, now);
  const bool epas_fresh = tesla_preap_source_fresh(preap_epas_seen, preap_epas_ts, now);
  const bool di_brake_fresh = tesla_preap_source_fresh(preap_di_brake_seen, preap_di_brake_ts, now);
  const bool brake_message_fresh = tesla_preap_source_fresh(preap_brake_message_seen, preap_brake_message_ts, now);
  const bool sources_fresh = gear_fresh && doors_fresh && epas_fresh && di_brake_fresh && brake_message_fresh;
  return (preap_mode != PREAP_MODE_INVALID) && sources_fresh && preap_gear_drive && preap_doors_closed &&
         preap_epas_healthy;
}

static bool tesla_preap_required_sources_ready(uint32_t now) {
  const bool sources_valid = tesla_preap_required_sources_valid(now);
  return sources_valid && !preap_di_brake_pressed && !preap_brake_message_pressed;
}

static bool tesla_preap_calibration_window_open(uint32_t now) {
  const bool gear_fresh = tesla_preap_source_fresh(preap_gear_seen, preap_gear_ts, now);
  const bool di_brake_fresh = tesla_preap_source_fresh(preap_di_brake_seen, preap_di_brake_ts, now);
  const bool brake_message_fresh = tesla_preap_source_fresh(preap_brake_message_seen, preap_brake_message_ts, now);
  return preap_pedal_calibration && gear_fresh && di_brake_fresh && brake_message_fresh &&
         preap_gear_neutral && (preap_di_brake_pressed || preap_brake_message_pressed);
}

static void tesla_preap_revoke_calibration_authority(void) {
  controls_allowed = false;
  controls_allowed_lateral = false;
}

static void tesla_preap_request_lateral(void) {
  if (preap_pedal_calibration) {
    return;
  }
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
  } else if (addr == 0x214U) {
    checksum_byte = 2;
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
  } else if (msg->addr == 0x155U) {
    // ESP_B protects its speed and counter fields with an inverted sum.
    const uint8_t counter = tesla_preap_get_counter(msg);
    checksum = (uint8_t)(0xFFU - (0x0CU + ((uint32_t)counter << 4U) + msg->data[5] + msg->data[6]));
  } else if (checksum_byte >= 0) {
    // The gateway remaps these messages without changing their checksum seed.
    uint32_t checksum_address = msg->addr;
    if (msg->addr == 0x108U) {
      checksum_address = 0x106U;
    } else if (msg->addr == 0x118U) {
      checksum_address = 0x116U;
    } else if (msg->addr == 0x368U) {
      checksum_address = 0x256U;
    } else {
    }
    checksum = (uint8_t)((checksum_address & 0xFFU) + ((checksum_address >> 8) & 0xFFU));
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
    valid = (msg->data[7] & 0x3U) == 0x3U;
  }
  return valid;
}

static void preap_word_to_bytes(uint8_t *dst, uint32_t src) {
  dst[0] = (uint8_t)(src & 0xFFU);
  dst[1] = (uint8_t)((src >> 8U) & 0xFFU);
  dst[2] = (uint8_t)((src >> 16U) & 0xFFU);
  dst[3] = (uint8_t)((src >> 24U) & 0xFFU);
}

static bool preap_f190_payload_allowed(const CANPacket_t *msg) {
  static const uint8_t tester[8] = {0x02U, 0x3EU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t default_session[8] = {0x02U, 0x10U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t extended_session[8] = {0x02U, 0x10U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t read_f190[8] = {0x03U, 0x22U, 0xF1U, 0x90U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t flow_control[8] = {0x30U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t cleanup_marker[8] = {0x02U, 0x3EU, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  const uint8_t *allowed[] = {tester, default_session, extended_session, read_f190, flow_control, cleanup_marker};
  if (GET_LEN(msg) != 8U) {
    return false;
  }
  for (unsigned int i = 0U; i < (sizeof(allowed) / sizeof(allowed[0])); i++) {
    bool match = true;
    for (int b = 0; b < 8; b++) {
      if (msg->data[b] != allowed[i][b]) {
        match = false;
        break;
      }
    }
    if (match) {
      return true;
    }
  }
  return false;
}

static bool preap_f190_tx_ok(const CANPacket_t *msg) {
  // Read-only F190 on the radar bus. Never writes, routines, or security.
  if (!preap_radar_emulation || controls_allowed || controls_allowed_lateral) {
    return false;
  }
  if (msg->fd || (msg->bus != 1U)) {
    return false;
  }
  return preap_f190_payload_allowed(msg);
}

static uint32_t preap_radar_vin_char(int pos, int shift) {
  return ((uint32_t)preap_radar_vin[pos]) << (shift * 8);
}

static bool preap_radar_donor_active(void) {
  if (preap_radar_vin_complete != 7U) {
    return false;
  }
  // 0.6.6 default was 17 spaces. Treat that as "use this car."
  for (int i = 0; i < 17; i++) {
    if ((preap_radar_vin[i] != 0U) && (preap_radar_vin[i] != (uint8_t)' ')) {
      return true;
    }
  }
  return false;
}

static void preap_apply_radar_vin_msg(const CANPacket_t *msg) {
  const int rec = msg->data[0];
  if (rec == 0) {
    preap_radar_position = (msg->data[2] >> 1) & 0x03;
    preap_radar_epas_type = (msg->data[2] >> 3) & 0x07;
    preap_radar_vin[0] = msg->data[5];
    preap_radar_vin[1] = msg->data[6];
    preap_radar_vin[2] = msg->data[7];
    preap_radar_vin_complete |= 1U;
  } else if (rec == 1) {
    preap_radar_vin[3] = msg->data[1];
    preap_radar_vin[4] = msg->data[2];
    preap_radar_vin[5] = msg->data[3];
    preap_radar_vin[6] = msg->data[4];
    preap_radar_vin[7] = msg->data[5];
    preap_radar_vin[8] = msg->data[6];
    preap_radar_vin[9] = msg->data[7];
    preap_radar_vin_complete |= 2U;
  } else if (rec == 2) {
    preap_radar_vin[10] = msg->data[1];
    preap_radar_vin[11] = msg->data[2];
    preap_radar_vin[12] = msg->data[3];
    preap_radar_vin[13] = msg->data[4];
    preap_radar_vin[14] = msg->data[5];
    preap_radar_vin[15] = msg->data[6];
    preap_radar_vin[16] = msg->data[7];
    preap_radar_vin_complete |= 4U;
  } else {
  }
}

static const int preap_crc_lookup[256] = {
  0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53, 0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
  0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E, 0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
  0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4, 0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
  0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19, 0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
  0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40, 0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
  0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D, 0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
  0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7, 0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
  0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A, 0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
  0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75, 0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
  0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8, 0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
  0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2, 0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
  0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F, 0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
  0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66, 0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
  0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB, 0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
  0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1, 0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
  0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C, 0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4
};

static int preap_compute_crc8(uint32_t lo, uint32_t hi, int msg_len) {
  int crc = 0xFF;
  for (int x = 0; x < msg_len; x++) {
    int v = (x <= 3) ? ((int)((lo >> (x * 8)) & 0xFFU)) : ((int)((hi >> ((x - 4) * 8)) & 0xFFU));
    crc = preap_crc_lookup[crc ^ v];
  }
  return crc ^ 0xFF;
}

static void preap_radar_capture_tx(const CANPacket_t *pkt) {
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
  if (preap_radar_gtw_count < PREAP_RADAR_GTW_CAPTURE_MAX) {
    preap_radar_gtw_capture[preap_radar_gtw_count] = *pkt;
    preap_radar_gtw_count++;
  }
  if (pkt->addr == 0x2A9U) {
    preap_radar_car_config_capture = *pkt;
    preap_radar_car_config_captured = true;
  }
  if (pkt->addr == 0x2B9U) {
    preap_radar_vin_feed_capture = *pkt;
    preap_radar_vin_feed_captured = true;
  }
#else
  SAFETY_UNUSED(pkt);
#endif
}

static void preap_radar_send(CANPacket_t *pkt) {
  preap_radar_capture_tx(pkt);
#if defined(STM32H7) || defined(STM32F4)
  can_set_checksum(pkt);
  can_send(pkt, 1, true);
#else
  SAFETY_UNUSED(pkt);
#endif
}

static void preap_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
  CANPacket_t pkt = {0};
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1U;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  const int msg_len = (int)GET_LEN(src);
  for (int i = 0; i < msg_len; i++) {
    pkt.data[i] = src->data[i];
  }
  preap_radar_send(&pkt);
}

static void preap_transform_radar_car_config(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2A9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  lo = (lo & 0xFFFFF33FU) | 0x100U | 0x440U;
  hi = (hi & 0xCFFF0F0FU) | 0x10000000U | ((uint32_t)preap_radar_position << 4) | ((uint32_t)preap_radar_epas_type << 12);
  // Tinkla 0.6.6: VIN character 8 of '2' or '4' is dual-motor. Force 4WD
  // on the live 0x2A9 so xWD matches the donor VIN, not this Pre-AP chassis.
  if (preap_radar_donor_active() && ((preap_radar_vin[7] == (uint8_t)'2') ||
                                     (preap_radar_vin[7] == (uint8_t)'4'))) {
    lo |= 0x08U;
  }
  preap_word_to_bytes(&dst->data[0], lo);
  preap_word_to_bytes(&dst->data[4], hi);
}

static void preap_transform_radar_vin_feed(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2B9, .data_len_code = src->data_len_code};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  if (preap_radar_donor_active() && ((lo & 0x10U) == 0x10U)) {
    const int rec = (int)(lo & 0xFFU);
    if (rec == 0x10) {
      lo = (uint32_t)rec;
      hi = preap_radar_vin_char(0, 1) | preap_radar_vin_char(1, 2) | preap_radar_vin_char(2, 3);
    } else if (rec == 0x11) {
      lo = (uint32_t)rec | preap_radar_vin_char(3, 1) | preap_radar_vin_char(4, 2) | preap_radar_vin_char(5, 3);
      hi = preap_radar_vin_char(6, 0) | preap_radar_vin_char(7, 1) | preap_radar_vin_char(8, 2) | preap_radar_vin_char(9, 3);
    } else if (rec == 0x12) {
      lo = (uint32_t)rec | preap_radar_vin_char(10, 1) | preap_radar_vin_char(11, 2) | preap_radar_vin_char(12, 3);
      hi = preap_radar_vin_char(13, 0) | preap_radar_vin_char(14, 1) | preap_radar_vin_char(15, 2) | preap_radar_vin_char(16, 3);
    } else {
    }
  }
  preap_word_to_bytes(&dst->data[0], lo);
  preap_word_to_bytes(&dst->data[4], hi);
}

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  const uint8_t bus_num = to_fwd->bus;
  const uint32_t addr = to_fwd->addr;
  const uint8_t msg_len = (uint8_t)GET_LEN(to_fwd);

  if ((bus_num == 0U) && preap_radar_emulation) {
    if ((addr == 0x45U) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x219); }
    else if ((addr == 0x108U) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x109); }
    else if ((addr == 0x145U) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x149); }
    else if ((addr == 0x20AU) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x159); }
    else if ((addr == 0x308U) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x209); }
    else if ((addr == 0x30AU) && (msg_len == 8U)) { preap_radar_readdr(to_fwd, 0x2D9); }
    else if ((addr == 0x405U) && (msg_len == 8U)) {
      CANPacket_t vin_pkt = {0};
      preap_transform_radar_vin_feed(to_fwd, &vin_pkt);
      preap_radar_send(&vin_pkt);
    }
    else {
    }

    if ((addr == 0x398U) && (msg_len == 8U)) {
      CANPacket_t pkt = {0};
      preap_transform_radar_car_config(to_fwd, &pkt);
      preap_radar_send(&pkt);
    }

    if ((addr == 0x0EU) && (msg_len == 8U)) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x199, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      if (((lo >> 16) & 0xFF3FU) == 0xFF3FU) {
        lo = (lo & 0x0000FFFFU) | (0x0020U << 16);
        hi = (hi & 0x00FFFFF0U) | 0x00000004U;
        int crc = preap_compute_crc8(lo, hi, 7);
        hi = hi | ((uint32_t)crc << 24);
      }
      preap_word_to_bytes(&pkt.data[0], lo);
      preap_word_to_bytes(&pkt.data[4], hi);
      preap_radar_send(&pkt);
    }

    if ((addr == 0x115U) && (msg_len == 6U)) {
      preap_radar_readdr(to_fwd, 0x129);
      uint32_t hi_src = PREAP_GET_BYTES_48(to_fwd);
      int counter = ((int)(hi_src & 0xF0U) >> 4) & 0x0F;
      uint32_t syn_lo = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x1A9, .data_len_code = 5};
      preap_word_to_bytes(&pkt.data[0], syn_lo);
      pkt.data[4] = (uint8_t)cksm;
      preap_radar_send(&pkt);
    }

    if ((addr == 0x118U) && (msg_len == 6U)) {
      preap_radar_readdr(to_fwd, 0x119);
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t ws_counter = PREAP_GET_BYTES_48(to_fwd) & 0x0FU;
      uint32_t raw_speed = (0xFFF0000U & lo) >> 16U;
      uint32_t speed;
      if (raw_speed == 0xFFFU) {
        speed = 0x1FFFU;
      } else {
        int mph_x100 = ((int)raw_speed * 5) - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        speed = (kph_x100 < 0) ? 0U : (((uint32_t)kph_x100 / 4U) & 0x1FFFU);
      }
      uint32_t ws_lo = speed | (speed << 13U) | (speed << 26U);
      uint32_t ws_hi = ((speed >> 6U) | (speed << 7U) | (ws_counter << 20U)) & 0x00FFFFFFU;
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_lo & 0xFFU) + (int)((ws_lo >> 8) & 0xFFU) + (int)((ws_lo >> 16) & 0xFFU) + (int)((ws_lo >> 24) & 0xFFU)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_hi & 0xFFU) + (int)((ws_hi >> 8) & 0xFFU) + (int)((ws_hi >> 16) & 0xFFU)) & 0xFF;
      ws_hi = ws_hi | ((uint32_t)ws_cksm << 24);
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x169, .data_len_code = 8};
      preap_word_to_bytes(&pkt.data[0], ws_lo);
      preap_word_to_bytes(&pkt.data[4], ws_hi);
      preap_radar_send(&pkt);
    }
  }

  if ((bus_num == 1U) && preap_radar_emulation) {
    if ((addr == 0x631U) && (preap_radar_status == 0)) {
      preap_radar_status = 1;
      preap_last_radar_signal = microsecond_timer_get();
    }
    if ((addr == 0x300U) && (preap_radar_status == 1)) {
      preap_radar_status = 2;
      preap_last_radar_signal = microsecond_timer_get();
    }
  }
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
bool tesla_preap_radar_car_config_captured(void) {
  return preap_radar_car_config_captured;
}

uint32_t tesla_preap_radar_car_config_addr(void) {
  return preap_radar_car_config_capture.addr;
}

uint8_t tesla_preap_radar_car_config_bus(void) {
  return preap_radar_car_config_capture.bus;
}

uint8_t tesla_preap_radar_car_config_dlc(void) {
  return preap_radar_car_config_capture.data_len_code;
}

uint8_t tesla_preap_radar_car_config_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_car_config_capture.data[index];
}

bool tesla_preap_radar_vin_feed_captured(void) {
  return preap_radar_vin_feed_captured;
}

uint8_t tesla_preap_radar_vin_feed_data(int index) {
  if ((index < 0) || (index >= 8)) {
    return 0U;
  }
  return preap_radar_vin_feed_capture.data[index];
}

bool tesla_preap_radar_donor_active_debug(void) {
  return preap_radar_donor_active();
}

int tesla_preap_radar_gateway_count(void) {
  return preap_radar_gtw_count;
}

uint32_t tesla_preap_radar_gateway_addr(int index) {
  if ((index < 0) || (index >= preap_radar_gtw_count)) {
    return 0U;
  }
  return preap_radar_gtw_capture[index].addr;
}

uint8_t tesla_preap_radar_gateway_bus(int index) {
  if ((index < 0) || (index >= preap_radar_gtw_count)) {
    return 0U;
  }
  return preap_radar_gtw_capture[index].bus;
}

uint8_t tesla_preap_radar_gateway_dlc(int index) {
  if ((index < 0) || (index >= preap_radar_gtw_count)) {
    return 0U;
  }
  return preap_radar_gtw_capture[index].data_len_code;
}

bool tesla_preap_radar_gateway_fd(int index) {
  if ((index < 0) || (index >= preap_radar_gtw_count)) {
    return false;
  }
  return preap_radar_gtw_capture[index].fd;
}

uint8_t tesla_preap_radar_gateway_data(int index, int byte_index) {
  if ((index < 0) || (index >= preap_radar_gtw_count) || (byte_index < 0) || (byte_index >= 8)) {
    return 0U;
  }
  return preap_radar_gtw_capture[index].data[byte_index];
}

void tesla_preap_radar_gateway_reset(void) {
  preap_radar_gtw_count = 0;
  preap_radar_car_config_captured = false;
  preap_radar_vin_feed_captured = false;
}

void tesla_preap_observe_can(const CANPacket_t *msg) {
  tesla_preap_gtw_emulation(msg);
}
#endif

static bool tesla_preap_stock_cc_off_before_pull(uint32_t now) {
  if (preap_stock_cc_di_ts != now) {
    return !preap_stock_cc_di_engaged;
  }
  if (preap_stock_cc_di_engaged) {
    return false;
  }
  return preap_stock_cc_di_prior_valid && !preap_stock_cc_di_prior_engaged;
}

static void tesla_preap_process_first_pull(uint32_t now) {
  if (controls_allowed) {
    controls_allowed = false;
    tesla_preap_clear_stock_cc_confirmation();
    if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BUTTON);
    }
  } else if (preap_mode == PREAP_MODE_INDEPENDENT) {
    tesla_preap_request_lateral();
  } else {
  }
  preap_pull_pending = true;
  preap_first_pull_ts = now;
  if (!preap_enable_pedal) {
    preap_stock_cc_cancel_authorized = true;
    preap_stock_cc_cancel_sent = false;
    preap_stock_cc_cancel_sent_ts = 0U;
    preap_stock_cc_post_cancel_di = false;
    preap_stock_cc_pull2_latched = false;
    preap_stock_cc_pull2_ts = 0U;
    preap_stock_cc_awaiting_di_rise = false;
    preap_stock_cc_expected_counter = 0U;
    preap_stock_cc_reengage_authorized = false;
    preap_stock_cc_reengage_sent = false;
    stock_cc_reengage_confirmed = false;
  }
}

static void tesla_preap_process_second_pull(uint32_t now) {
  preap_pull_pending = false;
  if (preap_pedal_calibration) {
    tesla_preap_clear_pull_state();
    tesla_preap_revoke_calibration_authority();
    return;
  }
  const bool sources_ready = tesla_preap_required_sources_ready(now);
  const bool gas_fresh = tesla_preap_source_fresh(preap_gas_seen, preap_gas_ts, now);
  const bool engagement_ready = sources_ready && !gas_pressed && preap_gas_seen && gas_fresh;
  if (engagement_ready) {
    if (preap_enable_pedal) {
      controls_allowed = true;
      if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
        tesla_preap_request_lateral();
      }
    } else {
      if (tesla_preap_stock_cc_off_before_pull(now)) {
        preap_stock_cc_pull2_latched = true;
        preap_stock_cc_pull2_ts = now;
        stock_cc_reengage_confirmed = false;
      }
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

static void tesla_preap_revoke_stock_cc_longitudinal(void) {
  controls_allowed = false;
  tesla_preap_clear_stock_cc_confirmation();
  if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
    mads_exit_controls(MADS_DISENGAGE_REASON_LAG);
  } else {
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

  const bool sources_ready = tesla_preap_required_sources_ready(microsecond_timer_get());
  if (brake_released && preap_brake_paused_lateral && sources_ready) {
    preap_brake_paused_lateral = false;
    tesla_preap_request_lateral();
  }
}

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  const uint32_t now = microsecond_timer_get();
  if (msg->returned == 0U) {
    const bool is_pedal_sensor = (preap_enable_pedal || preap_pedal_calibration) && (msg->addr == 0x552U) && (msg->bus == preap_pedal_bus);
    if (is_pedal_sensor) {
      const int pedal_raw = (msg->data[0] << 8) | msg->data[1];
      const uint8_t pedal_state = msg->data[4] >> 4;
      const uint8_t pedal_counter = msg->data[4] & 0xFU;
      if (!preap_pedal_feedback_counter_seen || (pedal_counter != preap_pedal_feedback_counter)) {
        preap_pedal_feedback_counter_seen = true;
        preap_pedal_feedback_counter = pedal_counter;
        preap_pedal_feedback_advance_ts = now;
      }
      preap_pedal_feedback_healthy = pedal_state == 0U;
      preap_gas_seen = preap_pedal_feedback_healthy;
      preap_gas_ts = preap_pedal_feedback_advance_ts;
      gas_pressed = !preap_pedal_feedback_healthy || (pedal_raw > PREAP_PEDAL_GAS_THRESHOLD);
    }
    if (!is_pedal_sensor && (msg->bus == 0U)) {
      if (msg->addr == 0x370U) {
        const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
        const uint8_t hands_on_level = msg->data[4] >> 6;
        const uint8_t eac_status = msg->data[6] >> 5;
        const uint8_t eac_error_code = msg->data[2] >> 4;
        const bool epas_fault = (eac_status == 0U) && (eac_error_code >= 6U) && (eac_error_code <= 9U);
        const uint32_t hands_on_elapsed = safety_get_ts_elapsed(now, preap_hands_on_clear_ts);

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
          } else if (hands_on_elapsed >= PREAP_HANDS_ON_RESUME_US) {
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
        preap_gear_neutral = gear == PREAP_GEAR_NEUTRAL;
        preap_gear_ts = now;
        preap_di_brake_seen = brake_state <= 1U;
        preap_di_brake_pressed = ((msg->data[1] & 0x80U) != 0U) || (brake_state == 1U);
        preap_di_brake_ts = now;
        const bool brake_now = preap_brake_message_pressed || preap_di_brake_pressed;
        brake_pressed = brake_now;
        tesla_preap_apply_brake_policy(!brake_was_pressed && brake_now, brake_was_pressed && !brake_now);
        if (!preap_pedal_calibration && (!preap_gear_drive || !preap_di_brake_seen)) {
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
        const bool cruise_engaged = (cruise_state == 2U) || (cruise_state == 3U) || (cruise_state == 4U) ||
                                    (cruise_state == 6U) || (cruise_state == 7U);
        const bool cruise_rising = cruise_engaged && !preap_stock_cc_di_engaged;
        const bool cruise_falling = !cruise_engaged && preap_stock_cc_di_engaged;
        if (preap_stock_cc_di_seen && (now != preap_stock_cc_di_ts)) {
          preap_stock_cc_di_prior_engaged = preap_stock_cc_di_engaged;
          preap_stock_cc_di_prior_valid = true;
        }
        preap_stock_cc_di_engaged = cruise_engaged;
        preap_stock_cc_di_ts = now;
        preap_stock_cc_di_seen = true;
        const uint32_t confirmation_elapsed = safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts);
        const uint32_t cancel_elapsed = safety_get_ts_elapsed(now, preap_stock_cc_cancel_sent_ts);
        const bool sources_ready = tesla_preap_required_sources_ready(now);
        if (!preap_enable_pedal) {
          if (stock_cc_reengage_confirmed && cruise_falling) {
            tesla_preap_revoke_stock_cc_longitudinal();
          } else if (preap_stock_cc_cancel_sent && preap_stock_cc_post_cancel_di && cruise_engaged &&
                     !preap_stock_cc_reengage_sent) {
            if (preap_stock_cc_pull2_latched || preap_stock_cc_reengage_authorized) {
              tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
            } else {
              // A post-CANCEL OFF sample is no longer current. Do not let it
              // authorize a later SET after cruise has re-engaged.
              preap_stock_cc_post_cancel_di = false;
              preap_stock_cc_reengage_authorized = false;
            }
          } else if (preap_stock_cc_cancel_sent && !cruise_engaged && !preap_stock_cc_post_cancel_di) {
            if (cancel_elapsed >= PREAP_STOCK_CC_CONFIRM_US) {
              tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
            } else {
              preap_stock_cc_post_cancel_di = true;
            }
          } else {
          }
          if (preap_stock_cc_cancel_sent && !cruise_engaged && preap_stock_cc_post_cancel_di &&
              preap_stock_cc_pull2_latched && !preap_stock_cc_reengage_authorized &&
              (safety_get_ts_elapsed(now, preap_stock_cc_pull2_ts) > 0U)) {
            preap_stock_cc_reengage_authorized = true;
            preap_stock_cc_deadline_ts = now;
          }
          if (preap_stock_cc_reengage_sent) {
            if ((confirmation_elapsed < PREAP_STOCK_CC_CONFIRM_US) && cruise_rising && sources_ready &&
                preap_stock_cc_awaiting_di_rise) {
              controls_allowed = true;
              stock_cc_reengage_confirmed = true;
              stock_cc_reengage_counter = preap_stock_cc_expected_counter;
              tesla_preap_retire_confirmed_stock_cc_handshake();
              if (preap_mode == PREAP_MODE_CRUISE_COUPLED) {
                tesla_preap_request_lateral();
              }
            } else if (confirmation_elapsed >= PREAP_STOCK_CC_CONFIRM_US) {
              tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
            } else {
            }
          }
        }
      }

      if (msg->addr == 0x45U) {
        const uint8_t lever = msg->data[0] & 0x3FU;
        const uint8_t counter = tesla_preap_get_counter(msg);
        const bool counter_consecutive = !preap_stalk_counter_seen || (counter == ((preap_stalk_counter_last + 1U) & 0xFU));
        const bool sources_ready = tesla_preap_required_sources_ready(now);
        bool is_echo = false;
        if (preap_echo_active) {
          const uint32_t echo_elapsed = safety_get_ts_elapsed(now, preap_echo_ts);
          if (echo_elapsed > preap_echo_window_us) {
            preap_echo_active = false;
          } else if ((lever == preap_echo_lever) && (counter == preap_echo_counter)) {
            is_echo = true;
          } else {
          }
        }
        if (is_echo) {
          // Authorized TX echoes do not participate in the physical stalk sequence.
        } else {
          preap_stalk_counter_seen = true;
          preap_stalk_counter_last = counter;
          for (int i = 0; i < 8; i++) {
            preap_live_stw[i] = msg->data[i];
          }
          preap_live_stw_valid = true;
          if (lever == 1U) {
            tesla_preap_exit(MADS_DISENGAGE_REASON_BUTTON);
          } else if (!counter_consecutive) {
            preap_stalk_armed = false;
            preap_pull_pending = false;
            tesla_preap_revoke_stock_cc_longitudinal();
          } else if (lever == 0U) {
            if (sources_ready) {
              preap_stalk_armed = true;
            }
          } else if (lever == 2U) {
            if (preap_stalk_armed && sources_ready) {
              preap_stalk_armed = false;
              tesla_preap_process_main_pull(now);
            }
          } else if ((lever == PREAP_STALK_RES_ACCEL) || (lever == PREAP_STALK_RES_ACCEL_2ND) ||
                     (lever == PREAP_STALK_DECEL_SET) || (lever == PREAP_STALK_DECEL_2ND)) {
            // Direct stock-cruise +/- : disarm the edge, keep pending double-pull origin.
            preap_stalk_armed = false;
          } else {
            // Unknown stalk positions cannot carry engagement authority.
            preap_stalk_armed = false;
            preap_pull_pending = false;
          }
        }
      }

      const bool sources_valid = tesla_preap_required_sources_valid(now);
      if (!sources_valid &&
          (controls_allowed || controls_allowed_lateral || preap_pull_pending || preap_stock_cc_reengage_sent)) {
        preap_hands_on_clear_timing = false;
        tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
      }
    }
  }
  if (preap_pedal_calibration) {
    tesla_preap_revoke_calibration_authority();
  }
}

static void tesla_preap_invalid_rx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  preap_hands_on_clear_timing = false;
  tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
}

static void tesla_preap_tick(bool rx_checks_invalid) {
  const uint32_t now = microsecond_timer_get();
  const bool sources_valid = tesla_preap_required_sources_valid(now);
  const uint32_t confirmation_elapsed = safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts);
  tesla_preap_expire_unsent_cancel(now);
  if (rx_checks_invalid || !sources_valid) {
    preap_hands_on_clear_timing = false;
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
  if (preap_enable_pedal &&
      (!preap_pedal_feedback_healthy ||
       !preap_pedal_feedback_counter_seen ||
       (safety_get_ts_elapsed(now, preap_pedal_feedback_advance_ts) > PREAP_PEDAL_FEEDBACK_TIMEOUT_US))) {
    if ((preap_mode == PREAP_MODE_CRUISE_COUPLED) && controls_allowed) {
      tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
    } else {
      controls_allowed = false;
    }
  }
  if (!preap_enable_pedal && preap_stock_cc_cancel_sent && !preap_stock_cc_post_cancel_di &&
      (safety_get_ts_elapsed(now, preap_stock_cc_cancel_sent_ts) >= PREAP_STOCK_CC_CONFIRM_US)) {
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
  if ((preap_stock_cc_reengage_authorized || preap_stock_cc_reengage_sent) &&
      (confirmation_elapsed >= PREAP_STOCK_CC_CONFIRM_US)) {
    tesla_preap_exit(MADS_DISENGAGE_REASON_LAG);
  }
  if (preap_pedal_calibration) {
    tesla_preap_revoke_calibration_authority();
  }
}

static bool tesla_preap_stock_cc_tuple_ok(const CANPacket_t *msg, uint8_t lever) {
  bool ok = false;
  if ((msg->addr == 0x45U) && (msg->bus == 0U) && (msg->fd == false) && (GET_LEN(msg) == 8U) && preap_live_stw_valid) {
    const uint8_t counter = tesla_preap_get_counter(msg);
    const uint32_t checksum = tesla_preap_compute_checksum(msg);
    ok = ((msg->data[0] & 0x40U) != 0U) &&
         ((msg->data[0] & 0x80U) == 0U) &&
         ((msg->data[0] & 0x3FU) == lever) &&
         (checksum == tesla_preap_get_checksum(msg)) &&
         (counter == ((preap_stalk_counter_last + 1U) & 0xFU)) &&
         ((msg->data[6] & 0x07U) == (preap_live_stw[6] & 0x07U)) &&
         ((msg->data[6] & 0x08U) == 0U);
    for (int i = 1; i < 6; i++) {
      if (msg->data[i] != preap_live_stw[i]) {
        ok = false;
      }
    }
  }
  return ok;
}

static void tesla_preap_mark_echo(uint8_t lever, uint8_t counter, uint32_t window_us) {
  preap_echo_active = true;
  preap_echo_lever = lever;
  preap_echo_counter = counter;
  preap_echo_ts = microsecond_timer_get();
  preap_echo_window_us = window_us;
}

static bool tesla_preap_classic_tx_tuple(const CANPacket_t *msg, uint8_t bus, uint8_t len) {
  return (!msg->fd) && (msg->bus == bus) && (GET_LEN(msg) == len) &&
         (tesla_preap_get_checksum(msg) == tesla_preap_compute_checksum(msg));
}

static bool tesla_preap_lateral_actuation_allowed(void) {
  return controls_allowed_lateral && !steering_control_inhibited;
}

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  const AngleSteeringLimits PREAP_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };
  // Frozen NAP Pre-AP Model S VehicleModel: mass=2100+STD_CARGO_KG, wheelbase=2.960, steerRatio=15.0.
  const AngleSteeringParams PREAP_STEERING_PARAMS = {
    .slip_factor = -0.0005666,
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  bool allowed = false;

  // Host→panda donor VIN/config. Intercept; do not put 0x560 on the car.
  if (msg->addr == PREAP_RADAR_VIN_ADDR) {
    if ((msg->bus == 0U) && !msg->fd && (GET_LEN(msg) == 8U)) {
      preap_apply_radar_vin_msg(msg);
    }
    return false;
  }

  // Radar UDS on bus 1. Allow only the F190 read sequence while disengaged.
  if (msg->addr == PREAP_RADAR_UDS_ADDR) {
    return preap_f190_tx_ok(msg);
  }

  if (msg->addr == 0x551U) {
    if ((!(preap_enable_pedal || preap_pedal_calibration)) || (msg->bus != preap_pedal_bus) || msg->fd ||
        (GET_LEN(msg) != 6U)) {
      return false;
    }
    const bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
    const uint8_t counter = msg->data[4] & 0xFU;
    const int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
    const int raw_gas_cmd2 = (msg->data[2] << 8) | msg->data[3];
    const bool protocol_valid = ((msg->data[4] & 0x70U) == 0U) &&
                                (tesla_preap_get_checksum(msg) == tesla_preap_compute_checksum(msg)) &&
                                (raw_gas_cmd < 65535) && (raw_gas_cmd2 < 65535) &&
                                (!preap_pedal_tx_counter_seen ||
                                 (counter == ((preap_pedal_tx_counter + 1U) & 0xFU)));
    if (!protocol_valid) {
      return false;
    }
    if (pedal_enable) {
      if (preap_pedal_calibration) {
        if (!tesla_preap_calibration_window_open(microsecond_timer_get())) {
          return false;
        }
      } else {
        const uint32_t feedback_age = safety_get_ts_elapsed(microsecond_timer_get(), preap_pedal_feedback_advance_ts);
        if (!get_longitudinal_allowed() || gas_pressed || preap_di_brake_pressed || preap_brake_message_pressed ||
            !preap_pedal_feedback_healthy || !preap_pedal_feedback_counter_seen ||
            (feedback_age > PREAP_PEDAL_FEEDBACK_TIMEOUT_US)) {
          return false;
        }
      }
    } else if ((raw_gas_cmd > 500) || (raw_gas_cmd2 > 500)) {
      return false;
    }
    preap_pedal_tx_counter_seen = true;
    preap_pedal_tx_counter = counter;
    return true;
  }
  if (msg->addr == 0x488U) {
    if (!tesla_preap_classic_tx_tuple(msg, 0U, 4U)) {
      return false;
    }
    // DAS_steeringHapticRequest is unused. Reject nonzero regardless of control type or permission.
    if ((msg->data[0] & 0x80U) != 0U) {
      return false;
    }
    const int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    const int desired_angle = raw_angle_can - 16384;
    const int steer_control_type = msg->data[2] >> 6;
    const bool steer_control_enabled = steer_control_type == 1;
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      return false;
    }
    if (steer_control_enabled && !tesla_preap_lateral_actuation_allowed()) {
      return false;
    }
    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, PREAP_STEERING_LIMITS, PREAP_STEERING_PARAMS)) {
      return false;
    }
    return true;
  }
  if (msg->addr == 0x214U) {
    if (!tesla_preap_classic_tx_tuple(msg, 0U, 3U)) {
      return false;
    }
    const int epas_control_type = msg->data[0] & 0x07U;
    if (epas_control_type > 1) {
      return false;
    }
    if ((epas_control_type == 1) && !tesla_preap_lateral_actuation_allowed()) {
      return false;
    }
    return true;
  }
  if (msg->addr == 0x3E9U) {
    if (!tesla_preap_classic_tx_tuple(msg, 0U, 8U)) {
      return false;
    }
    const int turn_req = msg->data[1] & 0x03U;
    if (turn_req > 3) {
      return false;
    }
    if (!tesla_preap_lateral_actuation_allowed()) {
      return false;
    }
    return true;
  }
  if (!preap_enable_pedal && (msg->addr == 0x45U)) {
    const uint32_t now = microsecond_timer_get();
    tesla_preap_expire_unsent_cancel(now);
    const uint8_t lever = msg->data[0] & 0x3FU;
    const uint8_t counter = tesla_preap_get_counter(msg);
    if ((lever == 1U) && tesla_preap_cancel_window_open(now) &&
        tesla_preap_stock_cc_tuple_ok(msg, 1U)) {
      preap_stock_cc_cancel_sent = true;
      preap_stock_cc_cancel_sent_ts = now;
      tesla_preap_mark_echo(1U, counter, PREAP_CANCEL_ECHO_US);
      allowed = true;
    } else if ((lever == 16U) && preap_stock_cc_reengage_authorized &&
               (safety_get_ts_elapsed(now, preap_stock_cc_deadline_ts) < PREAP_STOCK_CC_CONFIRM_US) &&
               preap_stock_cc_cancel_sent &&
               preap_stock_cc_post_cancel_di && !preap_stock_cc_reengage_sent &&
               tesla_preap_stock_cc_tuple_ok(msg, 16U)) {
      preap_stock_cc_reengage_sent = true;
      preap_stock_cc_awaiting_di_rise = true;
      preap_stock_cc_expected_counter = (uint8_t)((stock_cc_reengage_counter + 1U) & 0xFFU);
      preap_stock_cc_deadline_ts = now;
      tesla_preap_mark_echo(16U, counter, PREAP_SPOOF_ECHO_US);
      allowed = true;
    } else {
    }
  }
  return allowed;
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
  static RxCheck preap_rx_checks_with_pedal_bus_0[] = {
    {.msg = {{0x370, 0, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .max_counter = 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x20A, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x45, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    {.msg = {{0x552, 0, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  static RxCheck preap_rx_checks_with_pedal_bus_2[] = {
    {.msg = {{0x370, 0, 8, 25U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .max_counter = 7U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x20A, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x45, 0, 8, 10U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    {.msg = {{0x552, 2, 6, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL);
  preap_pedal_calibration = GET_FLAG(param, PREAP_FLAG_PEDAL_CALIBRATION);
  preap_pedal_bus = GET_FLAG(param, PREAP_FLAG_PEDAL_BUS_ZERO) ? 0U : 2U;
  preap_mode = (uint8_t)(current_safety_param_sp & PREAP_MODE_MASK);
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION);
  preap_radar_position = 0;
  preap_radar_epas_type = 0;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_radar_vin_complete = 0;
  for (int i = 0; i < 17; i++) {
    preap_radar_vin[i] = (uint8_t)' ';
  }
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
  preap_radar_car_config_captured = false;
  preap_radar_vin_feed_captured = false;
  preap_radar_gtw_count = 0;
#endif

  preap_gear_seen = false;
  preap_gear_drive = false;
  preap_gear_neutral = false;
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
  preap_pedal_feedback_counter_seen = false;
  preap_pedal_feedback_counter = 0U;
  preap_pedal_feedback_advance_ts = 0U;
  preap_pedal_feedback_healthy = false;
  preap_pedal_tx_counter_seen = false;
  preap_pedal_tx_counter = 0U;
  preap_brake_paused_lateral = false;
  preap_hands_on_clear_timing = false;
  preap_hands_on_clear_ts = 0U;
  steering_control_inhibited = false;
  stock_cc_reengage_counter = 0U;
  stock_cc_reengage_confirmed = false;
  preap_stalk_counter_seen = false;
  preap_stalk_counter_last = 0U;
  preap_live_stw_valid = false;
  preap_stock_cc_di_engaged = false;
  preap_stock_cc_di_ts = 0U;
  preap_stock_cc_di_seen = false;
  preap_stock_cc_di_prior_engaged = false;
  preap_stock_cc_di_prior_valid = false;
  tesla_preap_clear_pull_state();

  safety_config ret = {
    .rx_checks = preap_rx_checks,
    .rx_checks_len = 8,
    .tx_msgs = NULL,
    .tx_msgs_len = 0,
    .disable_forwarding = true,
  };
  if (preap_pedal_calibration) {
    ret.rx_checks = preap_pedal_bus == 0U ? preap_rx_checks_with_pedal_bus_0 : preap_rx_checks_with_pedal_bus_2;
    ret.rx_checks_len = 9;
    static CanMsg PREAP_TX_MSGS_PEDAL_CALIBRATION_BUS_0[] = {
      {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},
    };
    static CanMsg PREAP_TX_MSGS_PEDAL_CALIBRATION_BUS_2[] = {
      {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},
    };
    ret.tx_msgs = preap_pedal_bus == 0U ? PREAP_TX_MSGS_PEDAL_CALIBRATION_BUS_0 : PREAP_TX_MSGS_PEDAL_CALIBRATION_BUS_2;
    ret.tx_msgs_len = 1;
  } else if (preap_enable_pedal) {
    ret.rx_checks = preap_pedal_bus == 0U ? preap_rx_checks_with_pedal_bus_0 : preap_rx_checks_with_pedal_bus_2;
    ret.rx_checks_len = 9;
    static CanMsg PREAP_TX_MSGS_PEDAL_BUS_0[] = {
      {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},
      {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},
      {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},
      {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},
      {0x560, 0, 8, .check_relay = false, .disable_static_blocking = true},  // donor VIN/config to panda
      {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // radar F190 read
    };
    static CanMsg PREAP_TX_MSGS_PEDAL_BUS_2[] = {
      {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},
      {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},
      {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},
      {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},
      {0x560, 0, 8, .check_relay = false, .disable_static_blocking = true},  // donor VIN/config to panda
      {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // radar F190 read
    };
    ret.tx_msgs = preap_pedal_bus == 0U ? PREAP_TX_MSGS_PEDAL_BUS_0 : PREAP_TX_MSGS_PEDAL_BUS_2;
    ret.tx_msgs_len = 6;
  } else {
    static CanMsg PREAP_TX_MSGS_STOCK_CC[] = {
      {0x45, 0, 8, .check_relay = false, .disable_static_blocking = true},
      {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},
      {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},
      {0x3E9, 0, 8, .check_relay = false, .disable_static_blocking = true},
      {0x560, 0, 8, .check_relay = false, .disable_static_blocking = true},  // donor VIN/config to panda
      {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // radar F190 read
    };
    ret.tx_msgs = PREAP_TX_MSGS_STOCK_CC;
    ret.tx_msgs_len = 6;
  }
  return ret;
}

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .rx_all = tesla_preap_gtw_emulation,  // must see ALL CAN traffic for radar GTW forwarding
  .invalid_rx = tesla_preap_invalid_rx_hook,
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .tick = tesla_preap_tick,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_counter = tesla_preap_get_counter,
  .get_quality_flag_valid = tesla_preap_get_quality_flag_valid,
};
