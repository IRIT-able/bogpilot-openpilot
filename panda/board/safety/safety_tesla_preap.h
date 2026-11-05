#pragma once

// =============================================================================
// SAFETY_TESLA_PREAP — Pre-Autopilot Tesla Model S (BogPilot)
// =============================================================================
//
// Adapted from NotAutopilot's opendbc safety/modes/tesla_preap.h for BogPilot's
// FrogPilot 0.9.7 panda API. Credit: NotAutopilot (nap-release / tesla_preap).
// Spiritual predecessor: Tinkla (BogGyver) Pre-AP safety tables.
//
// This mode is COMPLETELY SEPARATE from safety_tesla.h (SAFETY_TESLA / AP1).
// Selecting SAFETY_TESLA_PREAP never changes TESLA_AP1_TX_MSGS or tesla_hooks.
//
// Pre-AP has no Autopilot ECU and no harness relay. generic_rx_checks is called
// with stock_ecu_detected=false (same idea as Tinkla "PreAP has no relay").
//
// Bosch radar GTW emulation (this cut): when PREAP_FLAG_RADAR_EMULATION is set,
// rx_all readdresses chassis bus-0 frames onto bus 1 and synthesizes the wheel-
// speed / ESP / carConfig messages the retrofit radar needs to wake and track.
// Firmware TX uses can_send(..., skip_tx_hook=true). Python may still send a
// dummy VIN (0x560) on bus 1 via the tight TX whitelist. Default off.
// PANDA FIRMWARE REBUILD REQUIRED (mode 32 + this rx_all). Source-only commit;
// integrator merges into bogpilot-na-preap-combined and builds fw once.
//
// License: MIT (Comma.ai / openpilot lineage); keep notices with redistributions.
// =============================================================================

const int PREAP_FLAG_ENABLE_PEDAL = 1;
const int PREAP_FLAG_RADAR_EMULATION = 2;       // GTW readdr + radar-bus helper TX (default off)
const int PREAP_FLAG_RADAR_BEHIND_NOSECONE = 4;  // radar_position bit for GTW_carConfig

// Same physical car as AP1 Model S. Rate tables match TESLA_AP1_STEERING_LIMITS
// so Pre-AP does not invent a looser steer envelope than AP1.
const SteeringLimits TESLA_PREAP_STEERING_LIMITS = {
  .angle_deg_to_can = 10,
  .angle_rate_up_lookup = {
    {2., 7., 17.},
    {8., 4., 2.5}
  },
  .angle_rate_down_lookup = {
    {2., 7., 17.},
    {9., 5., 4.5}
  },
};

static bool preap_enable_pedal = false;
static bool preap_radar_emulation = false;
static bool preap_radar_behind_nosecone = false;
static int preap_pedal_can = -1;
static int preap_gear = 4;
static int preap_gear_prev = 4;
static bool preap_doors_open = false;
static uint32_t preap_last_stalk_engage_us = 0;
#define PREAP_CANCEL_ECHO_WINDOW_US 600000U

// Radar GTW state (NAP tesla_preap)
static int preap_radar_status = 0;
static uint32_t preap_last_radar_signal = 0;
static int preap_radar_epas_type = 0;
static int preap_radar_position = 0;

#if defined(STM32H7) || defined(STM32F4)
void can_send(CANPacket_t *to_push, uint8_t bus_number, bool skip_tx_hook);
void can_set_checksum(CANPacket_t *packet);
#endif

#define PREAP_GET_BYTES_04(msg) ((msg)->data[0] | ((msg)->data[1] << 8) | ((msg)->data[2] << 16) | ((msg)->data[3] << 24))
#define PREAP_GET_BYTES_48(msg) ((msg)->data[4] | ((msg)->data[5] << 8) | ((msg)->data[6] << 16) | ((msg)->data[7] << 24))
#define PREAP_WORD_TO_BYTES(dst8, src32) 0[dst8] = ((src32) & 0xFFU); 1[dst8] = (((src32) >> 8U) & 0xFFU); 2[dst8] = (((src32) >> 16U) & 0xFFU); 3[dst8] = (((src32) >> 24U) & 0xFFU)



// Narrow TX surface from NotAutopilot tesla_preap (no HUD spam, no iBooster).
const CanMsg TESLA_PREAP_TX_MSGS[] = {
  {0x488, 0, 4},  // DAS_steeringControl
  {0x2b9, 0, 8},  // DAS_control
  {0x214, 0, 3},  // EPB_epasControl
  {0x551, 0, 6},  // Pedal GAS_COMMAND bus 0
  {0x551, 2, 6},  // Pedal GAS_COMMAND bus 2
  {0x45,  0, 8},  // STW_ACTN_RQ stalk spoof
};

// Extra TX when PREAP_FLAG_RADAR_EMULATION is on (radar bus = 1).
// Firmware GTW frames use can_send(..., true) and skip this whitelist.
// Python dummy-VIN / optional GTW helpers still need these entries.
const CanMsg TESLA_PREAP_TX_MSGS_WITH_RADAR[] = {
  {0x488, 0, 4},
  {0x2b9, 0, 8},
  {0x214, 0, 3},
  {0x551, 0, 6},
  {0x551, 2, 6},
  {0x45,  0, 8},
  {0x560, 1, 8},  // VIN_VIP dummy/donor (Python)
  {0x2A9, 1, 8},  // GTW_carConfig helper (Python keepalive)
  {0x199, 1, 8},  // STW_ANGLHP helper
  {0x641, 1, 8},  // Radar UDS F190 read-only (exact payloads in tx_hook)
};

RxCheck tesla_preap_rx_checks[] = {
  {.msg = {{0x370, 0, 8, .frequency = 25U}, { 0 }, { 0 }}},   // EPAS_sysStatus
  {.msg = {{0x108, 0, 8, .frequency = 100U}, { 0 }, { 0 }}},  // DI_torque1
  {.msg = {{0x118, 0, 6, .frequency = 100U}, { 0 }, { 0 }}},  // DI_torque2
  {.msg = {{0x20a, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // BrakeMessage
  {.msg = {{0x368, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},   // DI_state
  {.msg = {{0x318, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},   // GTW_carState
  {.msg = {{0x45,  0, 8, .frequency = 10U}, { 0 }, { 0 }}},   // STW_ACTN_RQ
  {.msg = {{0x155, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // ESP_B
};

RxCheck tesla_preap_rx_checks_with_pedal[] = {
  {.msg = {{0x370, 0, 8, .frequency = 25U}, { 0 }, { 0 }}},
  {.msg = {{0x108, 0, 8, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{0x118, 0, 6, .frequency = 100U}, { 0 }, { 0 }}},
  {.msg = {{0x20a, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{0x368, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{0x318, 0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{0x45,  0, 8, .frequency = 10U}, { 0 }, { 0 }}},
  {.msg = {{0x155, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},
  {.msg = {{0x552, 0, 6, .frequency = 50U},
           {0x552, 2, 6, .frequency = 50U}, { 0 }}},  // GAS_SENSOR
};


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

// ============================================
// GTW Emulation: CAN0 → CAN1 for Bosch radar
// ============================================

static void tesla_preap_gtw_emulation(const CANPacket_t *to_fwd) {
  int bus_num = GET_BUS(to_fwd);
  int addr = GET_ADDR(to_fwd);

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
    if (addr == 0x398) {
      CANPacket_t pkt = {.returned = 0U, .rejected = 0U, .extended = to_fwd->extended,
                         .bus = 1, .addr = 0x2A9, .data_len_code = to_fwd->data_len_code};
      uint32_t lo = PREAP_GET_BYTES_04(to_fwd);
      uint32_t hi = PREAP_GET_BYTES_48(to_fwd);
      lo = (lo & 0xFFFFF33F) | 0x100 | 0x440;  // country=US, radar_type=Bosch
      hi = (hi & 0xCFFF0F0F) | 0x10000000 | (preap_radar_position << 4) | (preap_radar_epas_type << 12);
      PREAP_WORD_TO_BYTES(&pkt.data[0], lo);
      PREAP_WORD_TO_BYTES(&pkt.data[4], hi);
      // No data[7] checksum: BO_681 / GTW_carConfig has no checksum; byte7 holds
      // GTW_autopilot (0x10000000). Writing a sum would clobber autopilot=1.
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



static bool preap_lever_engage_prev = false;

static void tesla_preap_rx_hook(const CANPacket_t *msg) {
  // Pedal sensor may be on bus 0 or 2.
  if (preap_enable_pedal && (msg->addr == 0x552U)) {
    int pedal_val = (GET_BYTE(msg, 0) << 8) | GET_BYTE(msg, 1);
    gas_pressed = (pedal_val > 650);
    if (preap_pedal_can == -1) {
      preap_pedal_can = msg->bus;
    }
    return;
  }

  if (msg->bus != 0U) {
    return;
  }

  if (msg->addr == 0x370U) {
    int angle_meas_new = (((GET_BYTE(msg, 4) & 0x3FU) << 8) | GET_BYTE(msg, 5)) - 8192;
    update_sample(&angle_meas, angle_meas_new);

    int hands_on_level = GET_BYTE(msg, 4) >> 6;
    int eac_status = GET_BYTE(msg, 6) >> 5;
    int eac_error_code = GET_BYTE(msg, 2) >> 4;
    bool epas_rejecting = (eac_status == 0) && (eac_error_code >= 6) && (eac_error_code <= 9);
    if ((hands_on_level >= 3) || epas_rejecting) {
      controls_allowed = false;
      pcm_cruise_check(false);
    }
  }

  if (msg->addr == 0x155U) {
    float speed = ((((GET_BYTE(msg, 5) << 8) | GET_BYTE(msg, 6)) * 0.01f) * 0.277778f);
    UPDATE_VEHICLE_SPEED(speed);
    vehicle_moving = speed > 0.139f;
  }

  if (msg->addr == 0x108U) {
    if (!preap_enable_pedal) {
      gas_pressed = (GET_BYTE(msg, 6) != 0U);
    }
  }

  if (msg->addr == 0x20aU) {
    brake_pressed = false;
  }

  if (msg->addr == 0x118U) {
    preap_gear = (GET_BYTE(msg, 1) >> 4) & 0x07;
    if ((preap_gear_prev == 4) && (preap_gear != 4)) {
      controls_allowed = false;
    }
    preap_gear_prev = preap_gear;
  }

  if (msg->addr == 0x318U) {
    int d_fl = (GET_BYTE(msg, 1) >> 4) & 0x03;
    int d_fr = (GET_BYTE(msg, 1) >> 6) & 0x03;
    int d_rl = (GET_BYTE(msg, 2) >> 6) & 0x03;
    int d_rr = (GET_BYTE(msg, 3) >> 5) & 0x03;
    int d_ft = (GET_BYTE(msg, 6) >> 2) & 0x03;
    int d_tr = (GET_BYTE(msg, 5) >> 6) & 0x03;
    preap_doors_open = (d_fl == 1) || (d_fr == 1) || (d_rl == 1) || (d_rr == 1) || (d_ft == 1) || (d_tr == 1);
    if (preap_doors_open) {
      controls_allowed = false;
    }
  }

  if (msg->addr == 0x45U) {
    int lever = GET_BYTE(msg, 0) & 0x3FU;
    if (lever == 2) {
      if ((preap_gear == 4) && !preap_doors_open) {
        // controls_allowed can drop without clearing cruise_engaged_prev; a new pull must re-arm.
        if (!preap_lever_engage_prev) {
          cruise_engaged_prev = false;
        }
        pcm_cruise_check(true);
        preap_last_stalk_engage_us = microsecond_timer_get();
      }
    } else if (lever == 1) {
      uint32_t elapsed = get_ts_elapsed(microsecond_timer_get(), preap_last_stalk_engage_us);
      if (elapsed > PREAP_CANCEL_ECHO_WINDOW_US) {
        pcm_cruise_check(false);
      }
    }
    preap_lever_engage_prev = (lever == 2);
  }

  generic_rx_checks(false);
}


// Narrow F190 UDS allow (bus 1, addr 0x641). Exact NAP read-conversation bytes
// only — never WriteDataByIdentifier, never VIN-learn routines, never security.
#define PREAP_RADAR_UDS_ADDR 0x641U

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
  // Read-only F190 on the radar bus while disengaged. Never elevates to all-output.
  if (!preap_radar_emulation || controls_allowed) {
    return false;
  }
  if (msg->bus != 1U) {
    return false;
  }
  return preap_f190_payload_allowed(msg);
}

static bool tesla_preap_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  bool violation = false;

  // Radar UDS: allow only the exact F190 read conversation payloads.
  if (msg->addr == PREAP_RADAR_UDS_ADDR) {
    return preap_f190_tx_ok(msg);
  }

  if (msg->addr == 0x488U) {
    int raw_angle_can = ((GET_BYTE(msg, 0) & 0x7FU) << 8) | GET_BYTE(msg, 1);
    int desired_angle = raw_angle_can - 16384;
    int steer_control_type = GET_BYTE(msg, 2) >> 6;
    bool steer_control_enabled = (steer_control_type == 1);
    if (steer_angle_cmd_checks(desired_angle, steer_control_enabled, TESLA_PREAP_STEERING_LIMITS)) {
      violation = true;
    }
    if ((steer_control_type != 0) && (steer_control_type != 1)) {
      violation = true;
    }
  }

  if (msg->addr == 0x214U) {
    int epas_control_type = GET_BYTE(msg, 0) & 0x07U;
    if (epas_control_type > 1) {
      violation = true;
    }
  }

  if (msg->addr == 0x2b9U) {
    int aeb_event = GET_BYTE(msg, 2) & 0x03U;
    if (aeb_event != 0) {
      violation = true;
    }
  }

  if ((msg->addr == 0x560U) || (msg->addr == 0x2A9U) || (msg->addr == 0x199U)) {
    if (!preap_radar_emulation || (msg->bus != 1U)) {
      violation = true;
    }
  }

  if (msg->addr == 0x551U) {
    if (!preap_enable_pedal) {
      violation = true;
    } else {
      bool pedal_enable = (GET_BYTE(msg, 4) & 0x80U) != 0U;
      int raw_gas_cmd = (GET_BYTE(msg, 0) << 8) | GET_BYTE(msg, 1);
      if (pedal_enable) {
        if (!get_longitudinal_allowed()) {
          violation = true;
        }
      } else if (raw_gas_cmd > 500) {
        violation = true;
      }
    }
  }

  if (violation) {
    tx = false;
  }
  return tx;
}

static int tesla_preap_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  return -1;
}

static safety_config tesla_preap_init(uint16_t param) {
  preap_enable_pedal = GET_FLAG(param, PREAP_FLAG_ENABLE_PEDAL);
  preap_radar_emulation = GET_FLAG(param, PREAP_FLAG_RADAR_EMULATION);
  preap_radar_behind_nosecone = GET_FLAG(param, PREAP_FLAG_RADAR_BEHIND_NOSECONE);

  preap_gear = 4;
  preap_gear_prev = 4;
  preap_lever_engage_prev = false;
  preap_doors_open = false;
  preap_pedal_can = -1;
  preap_last_stalk_engage_us = 0;
  preap_radar_status = 0;
  preap_last_radar_signal = 0;
  preap_radar_position = preap_radar_behind_nosecone ? 1 : 0;

  if (preap_radar_emulation) {
    if (preap_enable_pedal) {
      return BUILD_SAFETY_CFG(tesla_preap_rx_checks_with_pedal, TESLA_PREAP_TX_MSGS_WITH_RADAR);
    }
    return BUILD_SAFETY_CFG(tesla_preap_rx_checks, TESLA_PREAP_TX_MSGS_WITH_RADAR);
  }
  if (preap_enable_pedal) {
    return BUILD_SAFETY_CFG(tesla_preap_rx_checks_with_pedal, TESLA_PREAP_TX_MSGS);
  }
  return BUILD_SAFETY_CFG(tesla_preap_rx_checks, TESLA_PREAP_TX_MSGS);
}

const safety_hooks tesla_preap_hooks = {
  .init = tesla_preap_init,
  .rx = tesla_preap_rx_hook,
  .tx = tesla_preap_tx_hook,
  .fwd = tesla_preap_fwd_hook,
  .rx_all = tesla_preap_gtw_emulation,
};
