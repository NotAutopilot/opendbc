#pragma once

// ============================================
// SAFETY_TESLA_PREAP — Pre-Autopilot Tesla Model S (2012-2014)
// ============================================
//
// Standalone safety mode for Pre-AP Tesla Model S. These cars have NO
// Autopilot ECU, NO harness relay, and a different EPAS/CAN layout than
// HW1+ Teslas. This is the spiritual successor to Tinkla (Boggyver's
// Pre-AP openpilot fork, tesla_unity_betaC3 branch).
//
// WHY check_relay=false AND disable_static_blocking=true:
//
//   Pre-AP has no harness relay hardware. Standard openpilot uses a relay
//   on the harness to switch between stock AP ECU and openpilot — when
//   openpilot is not active, the relay routes CAN to the stock ECU. On
//   Pre-AP there is no AP ECU and no relay; the panda connects directly
//   to the car's CAN bus. Setting check_relay=true would cause the panda
//   to falsely detect a "relay malfunction" and block ALL TX permanently.
//   disable_static_blocking=true is required for the same reason — without
//   a relay, the panda's static blocking logic (which assumes relay state)
//   would incorrectly block messages.
//
//   Tinkla handled this identically via generic_rx_checks(false) in the
//   older panda API, with the comment "PreAP has no relay" (safety_tesla.h
//   line 1071, tesla_unity_betaC3 branch). The modern API added check_relay
//   and disable_static_blocking with restrictive defaults, so we explicitly
//   set them to get the same behavior Tinkla had implicitly.
//
// WHY ignore_checksum=true AND ignore_counter=true on RX:
//
//   Pre-AP EPAS firmware uses a byte-sum checksum, but the exact algorithm
//   has not been fully verified against all firmware versions. A checksum
//   mismatch caused a silent 21-second steering dropout during testing.
//   Tinkla's RX checks also had no checksum/counter validation (frequency
//   set to 0 for all messages). Once the checksum algorithm is verified
//   across all Pre-AP EPAS firmware versions, these can be re-enabled.
//
// ALL ACTUAL SAFETY CHECKS REMAIN FULLY ACTIVE:
//   - Steering angle + rate limits via steer_angle_cmd_checks_vm()
//   - controls_allowed gating on all TX
//   - Disengage on hands-on override (level >= 3)
//   - Disengage on EPAS error codes 6-9
//   - Disengage on door open, gear out of Drive
//   - Disengage on stalk cancel (with 600ms echo filter)
//   - AEB events blocked from openpilot
//   - EPB_epasControl mode validation
//   - Pedal TX gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed()
//
// Completely independent from tesla_legacy.h — has its own hooks struct,
// counter/checksum functions, init, RX/TX/fwd hooks, and GTW emulation.
// Registered as SAFETY_TESLA_PREAP in declarations.h.

#include "opendbc/safety/declarations.h"

// Forward declarations for panda firmware CAN send (defined in can_common.h)
#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

// ============================================
// Byte manipulation macros
// ============================================

#define PREAP_GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define PREAP_GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define PREAP_WORD_TO_BYTES(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)

// ============================================
// Safety param flags
// ============================================
// Longitudinal is gated by PREAP_FLAG_ENABLE_PEDAL + get_longitudinal_allowed().
// There is no separate LONG_CONTROL flag — the framework's get_longitudinal_allowed()
// is a derived check (controls_allowed && !gas_pressed_prev), not a settable flag.
// This matches how tesla.h, honda.h, and hyundai.h handle longitudinal gating.

#define PREAP_FLAG_ENABLE_PEDAL         1U
#define PREAP_FLAG_RADAR_EMULATION      2U
#define PREAP_FLAG_RADAR_BEHIND_NOSECONE 4U
#define PREAP_FLAG_RADAR_DIAGNOSTIC     8U

// ============================================
// State variables
// ============================================

static bool preap_enable_pedal = false;
static bool preap_radar_emulation = false;
static bool preap_radar_behind_nosecone = false;
static bool preap_radar_diagnostic = false;

static int preap_pedal_can = -1;

// Gear and door checks
static int preap_gear = 4;        // init to Drive to avoid false disables on startup
static int preap_gear_prev = 4;
static bool preap_doors_open = false;

// Stalk echo filter
static uint32_t preap_last_stalk_engage_us = 0;
#define PREAP_CANCEL_ECHO_WINDOW_US 600000U  // 600ms

// Radar emulation state
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static uint32_t preap_radar_position = 0U;

typedef enum {
  PREAP_DIAG_IDLE,
  PREAP_DIAG_AWAIT_TESTER,
  PREAP_DIAG_DEFAULT_SESSION,
  PREAP_DIAG_AWAIT_DEFAULT_SESSION,
  PREAP_DIAG_EXTENDED_SESSION,
  PREAP_DIAG_AWAIT_EXTENDED_SESSION,
  PREAP_DIAG_READINESS,
  PREAP_DIAG_AWAIT_READINESS,
  PREAP_DIAG_READ_DID,
  PREAP_DIAG_AWAIT_READ_DID,
  PREAP_DIAG_READ_DTCS,
  PREAP_DIAG_AWAIT_READ_DTCS,
  PREAP_DIAG_READ_SNAPSHOT,
  PREAP_DIAG_AWAIT_READ_SNAPSHOT,
  PREAP_DIAG_READ_EXTENDED_DATA,
  PREAP_DIAG_AWAIT_READ_EXTENDED_DATA,
  PREAP_DIAG_CLEANUP,
  PREAP_DIAG_AWAIT_CLEANUP,
  PREAP_DIAG_POISONED,
} PreAPDiagnosticPhase;

#define PREAP_DIAG_INACTIVITY_TIMEOUT_US 30000000U
#define PREAP_DIAG_OVERALL_TIMEOUT_US 60000000U
#define PREAP_DIAG_CLEANUP_TIMEOUT_US 3000000U
#define PREAP_DIAG_MAX_DTC_COUNT 16U
#define PREAP_DIAG_MAX_DETAIL_RESPONSE_LENGTH 256U

static const uint16_t PREAP_DIAG_DIDS[] = {
  0xA022U, 0xF014U, 0xF015U, 0xF180U, 0xF181U, 0xF182U,
  0xF187U, 0xF188U, 0xF189U, 0xF18AU, 0xF18CU, 0xF191U,
  0xF192U, 0xF193U, 0xF194U, 0xF195U, 0xF197U, 0xF19EU,
};
#define PREAP_DIAG_DID_COUNT (sizeof(PREAP_DIAG_DIDS) / sizeof(PREAP_DIAG_DIDS[0]))

static PreAPDiagnosticPhase preap_diag_phase = PREAP_DIAG_IDLE;
static uint32_t preap_diag_started_at_us = 0U;
static uint32_t preap_diag_last_activity_us = 0U;
static uint8_t preap_diag_did_index = 0U;
static uint8_t preap_diag_dtc_count = 0U;
static uint8_t preap_diag_dtc_index = 0U;
static uint8_t preap_diag_dtc_codes[PREAP_DIAG_MAX_DTC_COUNT][3];
static uint8_t preap_diag_dtc_payload[3U + (PREAP_DIAG_MAX_DTC_COUNT * 4U)];
static bool preap_diag_attempt_consumed = false;
static bool preap_diag_flow_control_allowed = false;
static bool preap_diag_rx_in_progress = false;
static uint16_t preap_diag_rx_length = 0U;
static uint16_t preap_diag_rx_received = 0U;
static uint8_t preap_diag_rx_sequence = 1U;

static bool preap_diag_latched(void) {
  return preap_diag_phase != PREAP_DIAG_IDLE;
}

static void preap_diag_reset_rx(void) {
  preap_diag_flow_control_allowed = false;
  preap_diag_rx_in_progress = false;
  preap_diag_rx_length = 0U;
  preap_diag_rx_received = 0U;
  preap_diag_rx_sequence = 1U;
}

static void preap_diag_set_phase(PreAPDiagnosticPhase phase) {
  if ((preap_diag_phase == PREAP_DIAG_IDLE) && (phase != PREAP_DIAG_IDLE)) {
    preap_diag_started_at_us = microsecond_timer_get();
  }
  preap_diag_phase = phase;
  preap_diag_reset_rx();
}

static void preap_diag_note_activity(void) {
  preap_diag_last_activity_us = microsecond_timer_get();
}

static void preap_diag_poison(void) {
  if (preap_diag_latched()) {
    preap_diag_set_phase(PREAP_DIAG_POISONED);
    controls_allowed = false;
  }
}

static void preap_diag_guard(void) {
  if (!preap_diag_latched()) return;
  controls_allowed = false;
  uint32_t now = microsecond_timer_get();
  uint32_t timeout = (preap_diag_phase == PREAP_DIAG_AWAIT_CLEANUP) ? PREAP_DIAG_CLEANUP_TIMEOUT_US : PREAP_DIAG_INACTIVITY_TIMEOUT_US;
  if (((now - preap_diag_last_activity_us) >= timeout) || ((now - preap_diag_started_at_us) >= PREAP_DIAG_OVERALL_TIMEOUT_US)) {
    preap_diag_poison();
  }
}

static bool preap_diag_payload_matches(const CANPacket_t *msg, const uint8_t expected[8]) {
  bool matches = true;
  for (int index = 0; index < 8; index++) matches &= msg->data[index] == expected[index];
  return matches;
}

static bool preap_diag_did_matches(const CANPacket_t *msg) {
  uint16_t did = PREAP_DIAG_DIDS[preap_diag_did_index];
  return (msg->data[0] == 0x03U) && (msg->data[1] == 0x22U) && (msg->data[2] == (did >> 8U)) &&
         (msg->data[3] == (did & 0xFFU)) && (msg->data[4] == 0U) && (msg->data[5] == 0U) &&
         (msg->data[6] == 0U) && (msg->data[7] == 0U);
}

static bool preap_diag_detail_matches(const CANPacket_t *msg, uint8_t subfunction) {
  return (msg->data[0] == 0x06U) && (msg->data[1] == 0x19U) && (msg->data[2] == subfunction) &&
         (msg->data[3] == preap_diag_dtc_codes[preap_diag_dtc_index][0]) &&
         (msg->data[4] == preap_diag_dtc_codes[preap_diag_dtc_index][1]) &&
         (msg->data[5] == preap_diag_dtc_codes[preap_diag_dtc_index][2]) && (msg->data[6] == 0xFFU) &&
         (msg->data[7] == 0U);
}

static bool preap_diag_awaits_response(void) {
  return (preap_diag_phase == PREAP_DIAG_AWAIT_TESTER) || (preap_diag_phase == PREAP_DIAG_AWAIT_DEFAULT_SESSION) ||
         (preap_diag_phase == PREAP_DIAG_AWAIT_EXTENDED_SESSION) || (preap_diag_phase == PREAP_DIAG_AWAIT_READINESS) ||
         (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DID) || (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DTCS) ||
         (preap_diag_phase == PREAP_DIAG_AWAIT_READ_SNAPSHOT) || (preap_diag_phase == PREAP_DIAG_AWAIT_READ_EXTENDED_DATA) ||
         (preap_diag_phase == PREAP_DIAG_AWAIT_CLEANUP);
}

static uint8_t preap_diag_expected_sid(void) {
  switch (preap_diag_phase) {
    case PREAP_DIAG_AWAIT_TESTER:
    case PREAP_DIAG_AWAIT_READINESS: return 0x3EU;
    case PREAP_DIAG_AWAIT_DEFAULT_SESSION:
    case PREAP_DIAG_AWAIT_EXTENDED_SESSION:
    case PREAP_DIAG_AWAIT_CLEANUP: return 0x10U;
    case PREAP_DIAG_AWAIT_READ_DID: return 0x22U;
    case PREAP_DIAG_AWAIT_READ_DTCS:
    case PREAP_DIAG_AWAIT_READ_SNAPSHOT:
    case PREAP_DIAG_AWAIT_READ_EXTENDED_DATA: return 0x19U;
    default: return 0U;
  }
}

static bool preap_diag_prefix_valid(const uint8_t *payload, uint16_t length) {
  if (length < 2U) return false;
  switch (preap_diag_phase) {
    case PREAP_DIAG_AWAIT_TESTER:
    case PREAP_DIAG_AWAIT_READINESS: return (payload[0] == 0x7EU) && (payload[1] == 0x00U);
    case PREAP_DIAG_AWAIT_DEFAULT_SESSION:
    case PREAP_DIAG_AWAIT_CLEANUP: return (payload[0] == 0x50U) && (payload[1] == 0x01U);
    case PREAP_DIAG_AWAIT_EXTENDED_SESSION: return (payload[0] == 0x50U) && (payload[1] == 0x03U);
    case PREAP_DIAG_AWAIT_READ_DID: {
      uint16_t did = PREAP_DIAG_DIDS[preap_diag_did_index];
      return (length >= 3U) && (payload[0] == 0x62U) && (payload[1] == (did >> 8U)) && (payload[2] == (did & 0xFFU));
    }
    case PREAP_DIAG_AWAIT_READ_DTCS: return (payload[0] == 0x59U) && (payload[1] == 0x02U);
    case PREAP_DIAG_AWAIT_READ_SNAPSHOT:
    case PREAP_DIAG_AWAIT_READ_EXTENDED_DATA: {
      uint8_t subfunction = (preap_diag_phase == PREAP_DIAG_AWAIT_READ_SNAPSHOT) ? 0x04U : 0x06U;
      return (length >= 5U) && (payload[0] == 0x59U) && (payload[1] == subfunction) &&
             (payload[2] == preap_diag_dtc_codes[preap_diag_dtc_index][0]) &&
             (payload[3] == preap_diag_dtc_codes[preap_diag_dtc_index][1]) &&
             (payload[4] == preap_diag_dtc_codes[preap_diag_dtc_index][2]);
    }
    default: return false;
  }
}

static bool preap_diag_length_valid(uint16_t length) {
  switch (preap_diag_phase) {
    case PREAP_DIAG_AWAIT_TESTER:
    case PREAP_DIAG_AWAIT_READINESS: return length == 2U;
    case PREAP_DIAG_AWAIT_DEFAULT_SESSION:
    case PREAP_DIAG_AWAIT_EXTENDED_SESSION:
    case PREAP_DIAG_AWAIT_CLEANUP: return length >= 2U;
    case PREAP_DIAG_AWAIT_READ_DID: return length >= 3U;
    case PREAP_DIAG_AWAIT_READ_DTCS: return (length >= 3U) && (((length - 3U) % 4U) == 0U);
    case PREAP_DIAG_AWAIT_READ_SNAPSHOT:
    case PREAP_DIAG_AWAIT_READ_EXTENDED_DATA: return (length >= 6U) && (length <= PREAP_DIAG_MAX_DETAIL_RESPONSE_LENGTH);
    default: return false;
  }
}

static void preap_diag_store_dtc_inventory(uint16_t length) {
  uint16_t count = (length - 3U) / 4U;
  preap_diag_dtc_count = (count < PREAP_DIAG_MAX_DTC_COUNT) ? (uint8_t)count : PREAP_DIAG_MAX_DTC_COUNT;
  preap_diag_dtc_index = 0U;
  for (uint8_t index = 0U; index < preap_diag_dtc_count; index++) {
    uint16_t offset = 3U + ((uint16_t)index * 4U);
    preap_diag_dtc_codes[index][0] = preap_diag_dtc_payload[offset];
    preap_diag_dtc_codes[index][1] = preap_diag_dtc_payload[offset + 1U];
    preap_diag_dtc_codes[index][2] = preap_diag_dtc_payload[offset + 2U];
  }
}

static void preap_diag_advance_did(void) {
  preap_diag_did_index++;
  preap_diag_set_phase((preap_diag_did_index < PREAP_DIAG_DID_COUNT) ? PREAP_DIAG_READ_DID : PREAP_DIAG_READ_DTCS);
}

static void preap_diag_advance_detail(void) {
  preap_diag_dtc_index++;
  preap_diag_set_phase((preap_diag_dtc_index < preap_diag_dtc_count) ? PREAP_DIAG_READ_SNAPSHOT : PREAP_DIAG_CLEANUP);
}

static void preap_diag_finish_positive(uint16_t length) {
  if (!preap_diag_length_valid(length)) {
    preap_diag_poison();
    return;
  }
  switch (preap_diag_phase) {
    case PREAP_DIAG_AWAIT_TESTER: preap_diag_set_phase(PREAP_DIAG_DEFAULT_SESSION); break;
    case PREAP_DIAG_AWAIT_DEFAULT_SESSION: preap_diag_set_phase(PREAP_DIAG_EXTENDED_SESSION); break;
    case PREAP_DIAG_AWAIT_EXTENDED_SESSION:
      preap_diag_did_index = 0U;
      preap_diag_set_phase(PREAP_DIAG_READINESS);
      break;
    case PREAP_DIAG_AWAIT_READINESS: preap_diag_set_phase(PREAP_DIAG_READ_DID); break;
    case PREAP_DIAG_AWAIT_READ_DID: preap_diag_advance_did(); break;
    case PREAP_DIAG_AWAIT_READ_DTCS:
      preap_diag_store_dtc_inventory(length);
      preap_diag_set_phase((preap_diag_dtc_count > 0U) ? PREAP_DIAG_READ_SNAPSHOT : PREAP_DIAG_CLEANUP);
      break;
    case PREAP_DIAG_AWAIT_READ_SNAPSHOT: preap_diag_set_phase(PREAP_DIAG_READ_EXTENDED_DATA); break;
    case PREAP_DIAG_AWAIT_READ_EXTENDED_DATA: preap_diag_advance_detail(); break;
    case PREAP_DIAG_AWAIT_CLEANUP:
      preap_diag_set_phase(PREAP_DIAG_IDLE);
      break;
    default: preap_diag_poison(); break;
  }
}

static void preap_diag_handle_payload(const uint8_t *payload, uint16_t length) {
  if ((length == 3U) && (payload[0] == 0x7FU)) {
    if (payload[1] != preap_diag_expected_sid()) {
      preap_diag_poison();
    } else if (payload[2] == 0x78U) {
      preap_diag_note_activity();
    } else if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DID) {
      preap_diag_advance_did();
      preap_diag_note_activity();
    } else if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DTCS) {
      preap_diag_set_phase(PREAP_DIAG_CLEANUP);
      preap_diag_note_activity();
    } else if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_SNAPSHOT) {
      preap_diag_set_phase(PREAP_DIAG_READ_EXTENDED_DATA);
      preap_diag_note_activity();
    } else if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_EXTENDED_DATA) {
      preap_diag_advance_detail();
      preap_diag_note_activity();
    } else {
      preap_diag_poison();
    }
    return;
  }
  if (!preap_diag_prefix_valid(payload, length)) {
    preap_diag_poison();
    return;
  }
  if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DTCS) {
    uint16_t stored = (length < sizeof(preap_diag_dtc_payload)) ? length : sizeof(preap_diag_dtc_payload);
    for (uint16_t index = 0U; index < stored; index++) preap_diag_dtc_payload[index] = payload[index];
  }
  preap_diag_note_activity();
  preap_diag_finish_positive(length);
}

static void preap_diag_rx_frame(const CANPacket_t *msg) {
  if (!preap_diag_awaits_response()) {
    preap_diag_poison();
    return;
  }
  uint8_t frame_type = msg->data[0] >> 4U;
  if (frame_type == 0U) {
    uint8_t length = msg->data[0] & 0x0FU;
    if (preap_diag_rx_in_progress || (length == 0U) || (length > 7U)) preap_diag_poison();
    else preap_diag_handle_payload(&msg->data[1], length);
  } else if (frame_type == 1U) {
    uint16_t length = (((uint16_t)msg->data[0] & 0x0FU) << 8U) | msg->data[1];
    if (preap_diag_rx_in_progress || (length <= 7U) || !preap_diag_length_valid(length) || !preap_diag_prefix_valid(&msg->data[2], 6U)) {
      preap_diag_poison();
      return;
    }
    preap_diag_rx_in_progress = true;
    preap_diag_flow_control_allowed = true;
    preap_diag_rx_length = length;
    preap_diag_rx_received = 6U;
    preap_diag_rx_sequence = 1U;
    if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DTCS) {
      for (uint8_t index = 0U; index < 6U; index++) preap_diag_dtc_payload[index] = msg->data[index + 2U];
    }
    preap_diag_note_activity();
  } else if (frame_type == 2U) {
    uint8_t sequence = msg->data[0] & 0x0FU;
    if (!preap_diag_rx_in_progress || preap_diag_flow_control_allowed || (sequence != preap_diag_rx_sequence)) {
      preap_diag_poison();
      return;
    }
    uint16_t copied = ((preap_diag_rx_length - preap_diag_rx_received) < 7) ? (preap_diag_rx_length - preap_diag_rx_received) : 7;
    uint16_t offset = preap_diag_rx_received;
    preap_diag_rx_received += copied;
    if (preap_diag_phase == PREAP_DIAG_AWAIT_READ_DTCS) {
      for (uint16_t index = 0U; (index < copied) && ((offset + index) < sizeof(preap_diag_dtc_payload)); index++) {
        preap_diag_dtc_payload[offset + index] = msg->data[index + 1U];
      }
    }
    preap_diag_rx_sequence = (preap_diag_rx_sequence + 1U) & 0x0FU;
    preap_diag_note_activity();
    if (preap_diag_rx_received >= preap_diag_rx_length) {
      uint16_t length = preap_diag_rx_length;
      preap_diag_reset_rx();
      preap_diag_finish_positive(length);
    }
  } else {
    preap_diag_poison();
  }
}

static bool preap_diag_tx_frame(const CANPacket_t *msg) {
  static const uint8_t TESTER_PRESENT[8] = {0x02U, 0x3EU, 0x00U, 0U, 0U, 0U, 0U, 0U};
  static const uint8_t DEFAULT_SESSION[8] = {0x02U, 0x10U, 0x01U, 0U, 0U, 0U, 0U, 0U};
  static const uint8_t EXTENDED_SESSION[8] = {0x02U, 0x10U, 0x03U, 0U, 0U, 0U, 0U, 0U};
  static const uint8_t READ_DTCS[8] = {0x03U, 0x19U, 0x02U, 0xFFU, 0U, 0U, 0U, 0U};
  static const uint8_t FLOW_CONTROL[8] = {0x30U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  bool is_cleanup = preap_diag_payload_matches(msg, DEFAULT_SESSION);
  bool is_latched_recovery = preap_diag_latched() && is_cleanup &&
                             (preap_diag_phase != PREAP_DIAG_DEFAULT_SESSION) &&
                             (preap_diag_phase != PREAP_DIAG_CLEANUP) &&
                             (preap_diag_phase != PREAP_DIAG_AWAIT_CLEANUP);
  if (is_latched_recovery) {
    preap_diag_set_phase(PREAP_DIAG_AWAIT_CLEANUP);
    preap_diag_note_activity();
    return true;
  }
  bool allowed = false;
  switch (preap_diag_phase) {
    case PREAP_DIAG_IDLE:
      if (!controls_allowed && !preap_diag_attempt_consumed && preap_diag_payload_matches(msg, TESTER_PRESENT)) {
        preap_diag_attempt_consumed = true;
        preap_diag_set_phase(PREAP_DIAG_AWAIT_TESTER);
        allowed = true;
      } else if (!controls_allowed && is_cleanup) {
        preap_diag_attempt_consumed = true;
        preap_diag_set_phase(PREAP_DIAG_AWAIT_CLEANUP);
        allowed = true;
      }
      break;
    case PREAP_DIAG_DEFAULT_SESSION:
      if (is_cleanup) { preap_diag_set_phase(PREAP_DIAG_AWAIT_DEFAULT_SESSION); allowed = true; }
      break;
    case PREAP_DIAG_CLEANUP:
      if (is_cleanup) { preap_diag_set_phase(PREAP_DIAG_AWAIT_CLEANUP); allowed = true; }
      break;
    case PREAP_DIAG_EXTENDED_SESSION:
      if (preap_diag_payload_matches(msg, EXTENDED_SESSION)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_EXTENDED_SESSION); allowed = true; }
      break;
    case PREAP_DIAG_READINESS:
      if (preap_diag_payload_matches(msg, TESTER_PRESENT)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_READINESS); allowed = true; }
      break;
    case PREAP_DIAG_READ_DID:
      if (preap_diag_did_matches(msg)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_READ_DID); allowed = true; }
      break;
    case PREAP_DIAG_READ_DTCS:
      if (preap_diag_payload_matches(msg, READ_DTCS)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_READ_DTCS); allowed = true; }
      break;
    case PREAP_DIAG_READ_SNAPSHOT:
      if (preap_diag_detail_matches(msg, 0x04U)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_READ_SNAPSHOT); allowed = true; }
      break;
    case PREAP_DIAG_READ_EXTENDED_DATA:
      if (preap_diag_detail_matches(msg, 0x06U)) { preap_diag_set_phase(PREAP_DIAG_AWAIT_READ_EXTENDED_DATA); allowed = true; }
      break;
    default:
      if (preap_diag_payload_matches(msg, FLOW_CONTROL) && preap_diag_rx_in_progress && preap_diag_flow_control_allowed) {
        preap_diag_flow_control_allowed = false;
        allowed = true;
      }
      break;
  }
  if (allowed) {
    controls_allowed = false;
    preap_diag_note_activity();
  } else if (preap_diag_latched()) {
    preap_diag_poison();
  }
  return allowed;
}

// ============================================
// Checksum and counter (for EPAS validation)
// ============================================

static uint8_t tesla_preap_get_counter(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[6] & 0x0FU;  // EPAS_sysStatusCounter
  }
  return 0U;
}

static uint32_t tesla_preap_get_checksum(const CANPacket_t *msg) {
  if (msg->addr == 0x370U) {
    return msg->data[7];  // EPAS_sysStatusChecksum at byte 7
  }
  if (msg->addr == 0x488U) {
    return msg->data[3];  // DAS_steeringControlChecksum at byte 3
  }
  return 0U;
}

static uint32_t tesla_preap_compute_checksum(const CANPacket_t *msg) {
  // Tesla byte-sum checksum: sum of address bytes + all data bytes except checksum byte
  int checksum_byte = -1;
  if (msg->addr == 0x370U) {
    checksum_byte = 7;
  } else if (msg->addr == 0x488U) {
    checksum_byte = 3;
  }
  if (checksum_byte == -1) {
    return 0U;
  }

  uint8_t chksum = (uint8_t)(msg->addr & 0xFFU) + (uint8_t)((msg->addr >> 8) & 0xFFU);
  int len = GET_LEN(msg);
  for (int i = 0; i < len; i++) {
    if (i != checksum_byte) {
      chksum += msg->data[i];
    }
  }
  return chksum;
}

// CRC-8 lookup table (polynomial 0x1D) for steering angle re-addressing
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
    int v = (x <= 3) ? ((lo >> (x * 8)) & 0xFF) : ((hi >> ((x - 4) * 8)) & 0xFF);
    crc = preap_crc_lookup[crc ^ v];
  }
  return crc ^ 0xFF;
}

// ============================================
// GTW Emulation helpers
// ============================================

static void preap_radar_readdr(const CANPacket_t *src, uint16_t new_addr) {
#if defined(STM32H7) || defined(STM32F4)
  CANPacket_t pkt;
  pkt.returned = 0U;
  pkt.rejected = 0U;
  pkt.extended = src->extended;
  pkt.bus = 1;
  pkt.addr = new_addr;
  pkt.data_len_code = src->data_len_code;
  for (int i = 0; i < GET_LEN(src); i++) {
    pkt.data[i] = src->data[i];
  }
  can_set_checksum(&pkt);
  can_send(&pkt, 1, true);
#else
  (void)src;
  (void)new_addr;
#endif
}

static void preap_transform_radar_car_config(const CANPacket_t *src, CANPacket_t *dst) {
  *dst = (CANPacket_t){.returned = 0U, .rejected = 0U, .extended = src->extended,
                       .bus = 1, .addr = 0x2A9, .data_len_code = 8U};
  uint32_t lo = PREAP_GET_BYTES_04(src);
  uint32_t hi = PREAP_GET_BYTES_48(src);
  lo = (lo & 0xFFFFF33FU) | 0x100U | 0x440U;  // DAS hardware, Bosch radar, and park assist
  hi = (hi & 0xCFFF0F0FU) | 0x10000000U | (preap_radar_position << 4U);
  PREAP_WORD_TO_BYTES(&dst->data[0], lo);
  PREAP_WORD_TO_BYTES(&dst->data[4], hi);
}

#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
static bool preap_radar_car_config_captured = false;
static CANPacket_t preap_radar_car_config_capture;
#endif

// ============================================
// GTW Emulation: CAN0 → CAN1 for Bosch radar
// ============================================

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

  preap_diag_guard();
  if ((addr == 0x651U) && preap_diag_latched()) {
    if ((bus_num == 1) && (GET_LEN(to_fwd) == 8U)) {
      preap_diag_rx_frame(to_fwd);
    } else {
      preap_diag_poison();
    }
  }

  if (bus_num == 0 && preap_radar_emulation) {
    // Group A: Simple re-addresses
    switch (addr) {
      case 0x45:   preap_radar_readdr(to_fwd, 0x219); break;  // STW_ACTN_RQ
      case 0x108:  preap_radar_readdr(to_fwd, 0x109); break;  // DI_torque1
      case 0x145:  preap_radar_readdr(to_fwd, 0x149); break;  // ESP_145h
      case 0x20A:  preap_radar_readdr(to_fwd, 0x159); break;  // BrakeMessage -> ESP_C
      case 0x308:  preap_radar_readdr(to_fwd, 0x209); break;  // GTW_odo
      case 0x30A:  preap_radar_readdr(to_fwd, 0x2D9); break;  // BC_status
      case 0x405:  preap_radar_readdr(to_fwd, 0x2B9); break;  // VIP_405HS
      default: break;
    }

    // Group B: GTW_carConfig (0x398) → 0x2A9 with bitfield patching
    if ((addr == 0x398) && (GET_LEN(to_fwd) == 8U)) {
      CANPacket_t pkt;
      preap_transform_radar_car_config(to_fwd, &pkt);
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
      preap_radar_car_config_capture = pkt;
      preap_radar_car_config_captured = true;
#endif
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group B: STW_ANGLHP_STAT (0x0E) → 0x199 with SNA replacement
    if (addr == 0x0E) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x199, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      if (((lo >> 16) & 0xFF3F) == 0xFF3F) {
        lo = (lo & 0x00C0FFFF) | (0x0020 << 16);
        hi = (hi & 0x00FFFFF0) | 0x00000004;  // force DELPHI sensor ID
        int crc = preap_compute_crc8(lo, hi, 7);
        hi = hi | ((uint32_t)crc << 24);
      }
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: ESP_115h (0x115) → 0x129 + synthetic DI_espControl (0x1A9)
    if (addr == 0x115) {
      preap_radar_readdr(to_fwd, 0x129);
      uint32_t hi_src = PREAP_GET_BYTES_48(to_fwd);
      int counter = ((hi_src & 0xF0) >> 4) & 0x0F;
      uint32_t syn_lo = 0x000C0000U | ((uint32_t)counter << 28);
      int cksm = (0x38 + 0x0C + (counter << 4)) & 0xFF;
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x1A9, .data_len_code = 5};
      PREAP_WORD_TO_BYTES(&pkt.data[0], syn_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], (uint32_t)cksm);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }

    // Group C: DI_torque2 (0x118) → 0x119 + synthetic ESP_wheelSpeeds (0x169)
    if (addr == 0x118) {
      preap_radar_readdr(to_fwd, 0x119);
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      int ws_counter = PREAP_GET_BYTES_48(to_fwd) & 0x0F;
      int raw_speed = (int)((0xFFF0000U & lo) >> 16);
      int speed;
      if (raw_speed == 0xFFF) {
        speed = 0x1FFF;
      } else {
        int mph_x100 = raw_speed * 5 - 2500;
        int kph_x100 = mph_x100 * 1609 / 1000;
        speed = (kph_x100 < 0) ? 0 : ((kph_x100 / 4) & 0x1FFF);
      }
      uint32_t ws_lo = (uint32_t)(speed | (speed << 13) | (speed << 26));
      uint32_t ws_hi = (uint32_t)((speed >> 6) | (speed << 7) | (ws_counter << 20)) & 0x00FFFFFFU;
      int ws_cksm = 0x76;
      ws_cksm = (ws_cksm + (int)(ws_lo & 0xFF) + (int)((ws_lo >> 8) & 0xFF) + (int)((ws_lo >> 16) & 0xFF) + (int)((ws_lo >> 24) & 0xFF)) & 0xFF;
      ws_cksm = (ws_cksm + (int)(ws_hi & 0xFF) + (int)((ws_hi >> 8) & 0xFF) + (int)((ws_hi >> 16) & 0xFF)) & 0xFF;
      ws_hi = ws_hi | ((uint32_t)ws_cksm << 24);
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = 0,
                         .bus = 1, .addr = 0x169, .data_len_code = 8};
      PREAP_WORD_TO_BYTES(&pkt.data[0], ws_lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], ws_hi);
#if defined(STM32H7) || defined(STM32F4)
      can_set_checksum(&pkt);
      can_send(&pkt, 1, true);
#endif
    }
  }

  // Radar status tracking (CAN1 → informational only)
  if (bus_num == 1 && preap_radar_emulation) {
    if (addr == 0x631 && preap_radar_status == 0) {
      preap_radar_status = 1;
      preap_last_radar_signal = microsecond_timer_get();
    }
    if (addr == 0x300 && preap_radar_status == 1) {
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
#endif

// ============================================
// RX Hook
// ============================================

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // Pedal interceptor (0x552) — may arrive on bus 0 OR bus 2 depending on wiring.
  // Must be handled BEFORE the bus-0-only bailout below.
  // Whitelisted on both bus 0 and bus 2 in preap_rx_checks; the framework has
  // already verified the message matches one of them, so accept either here.
  //
  // Gas-press threshold: 650 raw, chosen from real Pre-AP drive data:
  //   - At-rest noise (driver not pressing): raw range 424-633, mean 470 (p99.9=602)
  //   - Actual gas press: raw range 441-1246, mean 799 (p10=607, p50=802)
  // The original threshold of 450 was inside the resting noise distribution and
  // caused false gas_pressed readings that blocked pedal TX → pedal wouldn't engage.
  // 650 gives zero false positives on rest noise while still catching the vast
  // majority of real driver presses. Python-layer DI_pedalPos is the primary
  // gas-override detection; the panda threshold here is a safety backstop.
  if (preap_enable_pedal && (msg->addr == 0x552U)) {
    int pedal_val = ((msg->data[0] << 8) | msg->data[1]);
    gas_pressed = (pedal_val > 650);
    if (preap_pedal_can == -1) {
      preap_pedal_can = msg->bus;
    }
    return;
  }

  // All other RX handlers are bus 0 only.
  if (msg->bus != 0U) return;

  // EPAS (0x370): steering angle, hands-on level, disengage detection
  if (msg->addr == 0x370U) {
    const int angle_meas_new = (((msg->data[4] & 0x3FU) << 8) | msg->data[5]) - 8192U;
    update_sample(&angle_meas, angle_meas_new);

    const int hands_on_level = msg->data[4] >> 6;
    const int eac_status = msg->data[6] >> 5;
    const int eac_error_code = msg->data[2] >> 4;

    // Disengage on hands-on override OR EPAS actively rejecting steering commands.
    // Error codes 6/7/8 = EPAS request validators rejected angle/rate, 9 = safety layer.
    // All indicate the EPAS stopped steering — driver must be notified immediately.
    bool epas_rejecting = (eac_status == 0) && (eac_error_code >= 6) && (eac_error_code <= 9);
    steering_disengage = (hands_on_level >= 3) || epas_rejecting;

    // Re-arm fix: force cruise_engaged_prev reset on steering disengage
    // so next stalk pull creates a clean rising edge
    if (steering_disengage && !steering_disengage_prev) {
      pcm_cruise_check(false);
    }
  }

  // Vehicle speed (ESP_B: 0x155) — derive vehicle_moving from actual speed
  if (msg->addr == 0x155U) {
    float speed = (((msg->data[5] << 8) | msg->data[6]) * 0.01f) * KPH_TO_MS;
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > (0.5f * KPH_TO_MS);
  }

  // Gas pressed from DI_torque1 (0x108) — only when pedal interceptor is not active.
  // (The pedal interceptor path is handled above the bus-0-only bailout since it may
  // arrive on bus 0 or bus 2.)
  if (msg->addr == 0x108U) {
    if (!preap_enable_pedal) {
      gas_pressed = msg->data[6] != 0U;
    }
  }

  // Brake (0x20a) — force false so generic_rx_checks doesn't drop controls_allowed.
  // Pre-AP brake → steering-only is handled in the software layer.
  if (msg->addr == 0x20aU) {
    brake_pressed = false;
  }

  // Cruise state (DI_state: 0x368) — vehicle_moving only, engagement via stalk
  if (msg->addr == 0x368U) {
    int cruise_state = (msg->data[1] >> 4) & 0x07U;
    // Backup vehicle_moving from cruise state (standstill detection)
    if (cruise_state == 3) {
      vehicle_moving = false;
    }
  }

  // Gear check (DI_torque2: 0x118) — disable controls on leaving Drive
  if (msg->addr == 0x118U) {
    preap_gear = (msg->data[1] >> 4) & 0x07;
    if ((preap_gear_prev == 4) && (preap_gear != 4)) {
      controls_allowed = false;
    }
    preap_gear_prev = preap_gear;
  }

  // Door check (GTW_carState: 0x318)
  if (msg->addr == 0x318U) {
    int d_fl = (msg->data[1] >> 4) & 0x03;
    int d_fr = (msg->data[1] >> 6) & 0x03;
    int d_rl = (msg->data[2] >> 6) & 0x03;
    int d_rr = (msg->data[3] >> 5) & 0x03;
    int d_ft = (msg->data[6] >> 2) & 0x03;
    int d_tr = (msg->data[5] >> 6) & 0x03;
    preap_doors_open = (d_fl == 1) || (d_fr == 1) || (d_rl == 1) || (d_rr == 1) || (d_ft == 1) || (d_tr == 1);
    if (preap_doors_open) {
      controls_allowed = false;
    }
  }

  // Stalk engagement (STW_ACTN_RQ: 0x45) with echo-filtered cancel
  if (msg->addr == 0x45U) {
    int lever = msg->data[0] & 0x3FU;
    if (lever == 2) {  // RWD = pull toward driver = enable
      if ((preap_gear == 4) && !preap_doors_open) {
        pcm_cruise_check(true);
        preap_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (lever == 1) {  // FWD = push away = cancel
      uint32_t elapsed = microsecond_timer_get() - preap_last_stalk_engage_us;
      if (elapsed > PREAP_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
      }
    }
  }
}

// ============================================
// TX Hook
// ============================================

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  preap_diag_guard();
  if (msg->addr == 0x641U) {
    return preap_radar_diagnostic && preap_diag_tx_frame(msg);
  }
  bool diagnostic_latched = preap_diag_latched();
  const AngleSteeringLimits PREAP_STEERING_LIMITS = {
    .max_angle = 3600,  // 360 deg, EPAS faults above this
    .angle_deg_to_can = 10,
    .frequency = 50U,
  };

  // Pre-AP Model S is physically the same car as HW1/HW2/HW3 Model S.
  // These values MUST match VehicleModel(TESLA_MODEL_S_HW3) in carcontroller.py.
  // Verified: mass=2100+STD_CARGO_KG, wheelbase=2.960, steerRatio=15.0
  //           → slip_factor = -0.0005666 (calc_slip_factor)
  // Confirmed by Lukas (xnor-tech, former comma employee, Tesla port author).
  const AngleSteeringParams PREAP_STEERING_PARAMS = {
    .slip_factor = -0.0005666,
    .steer_ratio = 15.,
    .wheelbase = 2.96,
  };

  bool tx = true;
  bool violation = false;

  // DAS_steeringControl (0x488)
  if (msg->addr == 0x488U) {
    int raw_angle_can = ((msg->data[0] & 0x7FU) << 8) | msg->data[1];
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = msg->data[2] >> 6;
    bool steer_control_enabled = steer_control_type == 1;

    if (steer_angle_cmd_checks_vm(desired_angle, steer_control_enabled, PREAP_STEERING_LIMITS, PREAP_STEERING_PARAMS)) {
      violation = true;
    }
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
  }

  // EPB_epasControl (0x214): only allow valid EAC modes (0=disable, 1=enable)
  if (msg->addr == 0x214U) {
    int epas_control_type = msg->data[0] & 0x07U;  // EPB_epasEACAllow: bits 2:0 of byte 0
    if (epas_control_type > 1) {
      violation = true;
    }
  }

  // DAS_control (0x2B9): no AEB events from openpilot
  if (msg->addr == 0x2B9U) {
    int aeb_event = msg->data[2] & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }
  }

  // Pedal interceptor (0x551 GAS_COMMAND): parse ENABLE bit and GAS_COMMAND
  // value to distinguish authoritative accel commands from driver-passthrough
  // release commands.
  //   DBC: SG_ ENABLE : 39|1@0+  →  bit 7 of data[4]
  //   DBC: SG_ GAS_COMMAND : 7|16@0+  →  bytes 0-1 big-endian (physical 0 = raw 450)
  //
  //   ENABLE=0: openpilot is releasing control. Comma Pedal ignores GAS_COMMAND
  //   and passes driver's OEM pedal voltage through. NAP's pedal passthrough
  //   feature sends this during driver gas override for a smooth handoff.
  //   Defense-in-depth: we still require the GAS_COMMAND raw value to be at or
  //   below the zero point (raw <= 500, which is ~2.5% physical) so a bugged or
  //   malicious ENABLE=0 + high-value message can't sneak through a potential
  //   Comma Pedal firmware bug.
  //
  //   ENABLE=1: authoritative actuation command. Gated by get_longitudinal_allowed()
  //   (controls_allowed && !gas_pressed_prev).
  if (msg->addr == 0x551U) {
    if (!preap_enable_pedal) {
      violation = true;
    } else {
      bool pedal_enable = (msg->data[4] & 0x80U) != 0U;
      int raw_gas_cmd = (msg->data[0] << 8) | msg->data[1];
      if (pedal_enable) {
        if (!get_longitudinal_allowed()) {
          violation = true;
        }
      } else {
        // ENABLE=0: only allow near-zero GAS_COMMAND values (defense-in-depth).
        // Legitimate passthrough sends physical 0 = raw 450.
        if (raw_gas_cmd > 500) {
          violation = true;
        }
      }
    }
  }

  if (diagnostic_latched) {
    violation = true;
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

// ============================================
// Forwarding Hook
// ============================================

static bool tesla_preap_fwd_hook(int bus_num, int addr) {
  preap_diag_guard();
  (void)bus_num;
  (void)addr;
  // Pre-AP has no AP ECU on bus 2. Block default 0↔2 forwarding to avoid
  // flooding a dead TX queue.
  return true;
}

// ============================================
// Init
// ============================================

static safety_config tesla_preap_init(uint16_t param) {
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL);
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION);
  preap_radar_behind_nosecone = GET_FLAG(param, PREAP_FLAG_RADAR_BEHIND_NOSECONE);
  preap_radar_diagnostic = GET_FLAG(param, PREAP_FLAG_RADAR_DIAGNOSTIC);
  preap_gear = 4;
  preap_gear_prev = 4;
  preap_doors_open = false;
  preap_pedal_can = -1;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_last_stalk_engage_us = 0;
  preap_radar_position = preap_radar_behind_nosecone ? 1U : 0U;
  preap_diag_phase = PREAP_DIAG_IDLE;
  preap_diag_started_at_us = 0U;
  preap_diag_last_activity_us = 0U;
  preap_diag_did_index = 0U;
  preap_diag_dtc_count = 0U;
  preap_diag_dtc_index = 0U;
  preap_diag_attempt_consumed = false;
  preap_diag_reset_rx();
#if defined(ALLOW_DEBUG) && !defined(STM32H7) && !defined(STM32F4)
  preap_radar_car_config_captured = false;
  preap_radar_car_config_capture = (CANPacket_t){0};
#endif

  // TX whitelist — no harness relay on Pre-AP
  static const CanMsg PREAP_TX_MSGS[] = {
    {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},  // DAS_steeringControl
    {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},  // DAS_control
    {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},  // EPB_epasControl
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 0
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},  // Pedal on bus 2
    {0x45,  0, 8, .check_relay = false, .disable_static_blocking = true},  // STW_ACTN_RQ (stalk spoof)
  };

  static const CanMsg PREAP_TX_MSGS_WITH_DIAGNOSTIC[] = {
    {0x488, 0, 4, .check_relay = false, .disable_static_blocking = true},
    {0x2B9, 0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x214, 0, 3, .check_relay = false, .disable_static_blocking = true},
    {0x551, 0, 6, .check_relay = false, .disable_static_blocking = true},
    {0x551, 2, 6, .check_relay = false, .disable_static_blocking = true},
    {0x45,  0, 8, .check_relay = false, .disable_static_blocking = true},
    {0x641, 1, 8, .check_relay = false, .disable_static_blocking = true},  // Radar diagnostics
  };

  // RX checks — disable EPAS counter/checksum until we verify the Pre-AP
  // EPAS firmware's checksum matches our compute_checksum exactly.
  // Mismatched validation caused silent 21s steering dropout.
  static RxCheck preap_rx_checks[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // EPAS_sysStatus
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque1
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},  // DI_torque2
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // BrakeMessage
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // DI_state
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // GTW_carState
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // STW_ACTN_RQ
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},   // ESP_B
  };

  // Pedal-enabled variant: adds 0x552 (GAS_SENSOR) to rx_checks so the
  // framework routes it to the rx hook. Split into its own array because
  // frequency=0 causes divide-by-zero in safety_tick (safety.h:330), which
  // marks the check as lagging and trips safetyRxChecksInvalid → controls
  // mismatch on cars without a pedal. 50Hz matches the Comma Pedal firmware.
  static RxCheck preap_rx_checks_with_pedal[] = {
    {.msg = {{0x370, 0, 8, 25U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x108, 0, 8, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x118, 0, 6, 100U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x20a, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x368, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x318, 0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x45,  0, 8, 10U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x155, 0, 8, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{0x552, 0, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true},
             {0x552, 2, 6, 50U, .ignore_quality_flag = true, .ignore_checksum = true, .ignore_counter = true}, { 0 }}},  // GAS_SENSOR
  };

  if (preap_enable_pedal) {
    return preap_radar_diagnostic ? BUILD_SAFETY_CFG(preap_rx_checks_with_pedal, PREAP_TX_MSGS_WITH_DIAGNOSTIC)
                                  : BUILD_SAFETY_CFG(preap_rx_checks_with_pedal, PREAP_TX_MSGS);
  }
  return preap_radar_diagnostic ? BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS_WITH_DIAGNOSTIC)
                                : BUILD_SAFETY_CFG(preap_rx_checks, PREAP_TX_MSGS);
}

// ============================================
// Hooks struct
// ============================================

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .rx_all = tesla_preap_gtw_emulation,  // must see ALL CAN traffic for radar GTW forwarding
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .get_counter = tesla_preap_get_counter,
  .get_checksum = tesla_preap_get_checksum,
  .compute_checksum = tesla_preap_compute_checksum,
  .get_quality_flag_valid = NULL,
};
