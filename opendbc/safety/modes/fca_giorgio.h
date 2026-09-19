#pragma once

#include "opendbc/safety/declarations.h"

#define FCA_GIORGIO_ABS_1           0xEEU
#define FCA_GIORGIO_ABS_3           0xFAU
#define FCA_GIORGIO_ENGINE_1        0xFCU
#define FCA_GIORGIO_EPS_2           0x106U
#define FCA_GIORGIO_LKA_COMMAND     0x1F6U
#define FCA_GIORGIO_LKA_COMMAND_2   0x117U
// TODO: this isn't actually an LKA message on promester, coming from radar ECU
#define FCA_GIORGIO_LKA_HUD_1       0x4AEU
#define FCA_GIORGIO_LKA_HUD_2       0x547U
#define FCA_GIORGIO_LKA_HUD_3       0x5A2U
#define FCA_GIORGIO_ACC_2           0x22AU

static uint8_t fca_giorgio_crc8_lut_j1850[256];  // Static lookup table for CRC8 SAE J1850

static safety_config fca_giorgio_init(uint16_t param) {
  SAFETY_UNUSED(param);

  // TODO: need to find a button message for cancel spam
  static const CanMsg FCA_GIORGIO_TX_MSGS[] = {
    {FCA_GIORGIO_LKA_COMMAND, 0, 4, .check_relay = true},
    {FCA_GIORGIO_LKA_COMMAND_2, 0, 4, .check_relay = true},
    {FCA_GIORGIO_LKA_HUD_1, 0, 8, .check_relay = true},
    {FCA_GIORGIO_LKA_HUD_2, 0, 8, .check_relay = true},
    {FCA_GIORGIO_LKA_HUD_3, 0, 8, .check_relay = true},
  };

  // TODO: need to find a message for driver gas
  // TODO: re-enable checksums/counters
  static RxCheck fca_giorgio_rx_checks[] = {
    {.msg = {{FCA_GIORGIO_ACC_2, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ABS_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ABS_3, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_ENGINE_1, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FCA_GIORGIO_EPS_2, 0, 7, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  gen_crc_lookup_table_8(0x1D, fca_giorgio_crc8_lut_j1850);
  return BUILD_SAFETY_CFG(fca_giorgio_rx_checks, FCA_GIORGIO_TX_MSGS);
}

static uint32_t fca_giorgio_get_checksum(const CANPacket_t *msg) {
  int checksum_byte = GET_LEN(msg) - 1U;
  return (uint8_t)msg->data[checksum_byte];
}

static uint8_t fca_giorgio_get_counter(const CANPacket_t *msg) {
  uint8_t counter;
  if (msg->addr == FCA_GIORGIO_ABS_3) {
    counter = (uint8_t)((msg->data[4] >> 3) & 0xFU);
  } else {
    int counter_byte = GET_LEN(msg) - 2U;
    counter = (uint8_t)msg->data[counter_byte] & 0xFU;
  }
  return counter;
}

static uint32_t fca_giorgio_compute_crc(const CANPacket_t *msg) {
  int len = GET_LEN(msg);

  // standard CRC-8 SAE J1850 (poly 0x1D, init 0xFF, final XOR 0xFF)
  uint8_t crc = 0xFFU;

  for (int i = 0; i < (len - 1); i++) {
    crc ^= (uint8_t)msg->data[i];
    crc = fca_giorgio_crc8_lut_j1850[crc];
  }

  return (uint8_t)(crc ^ 0xFFU);
}

static void fca_giorgio_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == 0U) {
    // Update in-motion state by sampling wheel speeds
    if (msg->addr == FCA_GIORGIO_ABS_1) {
      // Thanks, FCA, for these 13 bit signals. Makes perfect sense. Great work.
      // Signals: ABS_3.WHEEL_SPEED_[FL,FR,RL,RR]
      int wheel_speed_fl = (msg->data[1] >> 3) | (msg->data[0] << 5);
      int wheel_speed_fr = (msg->data[3] >> 6) | (msg->data[2] << 2) | ((msg->data[1] & 0x7U) << 10);
      int wheel_speed_rl = (msg->data[4] >> 1) | ((msg->data[3] & 0x3FU) << 7);
      int wheel_speed_rr = (msg->data[6] >> 4) | (msg->data[5] << 4) | ((msg->data[4] & 0x1U) << 12);
      vehicle_moving = (wheel_speed_fl + wheel_speed_fr + wheel_speed_rl + wheel_speed_rr) > 0;
    }

    // Update driver input torque samples
    // Signal: EPS_2.DRIVER_TORQUE
    if (msg->addr == FCA_GIORGIO_EPS_2) {
      int torque_driver_new = ((msg->data[2] << 3) | (msg->data[3] >> 5)) - 1024U;
      update_sample(&torque_driver, torque_driver_new);
    }

    // Signal: ENGINE_1.ACCEL_PEDAL
    if (msg->addr == FCA_GIORGIO_ENGINE_1) {
      gas_pressed = (((msg->data[2] & 0x1FU) << 3) | (msg->data[3] >> 5)) > 0U;
    }

    // Signal: ABS_3.BRAKE_PEDAL_SWITCH
    if (msg->addr == FCA_GIORGIO_ABS_3) {
      brake_pressed = GET_BIT(msg, 3U);
    }
  }

  if (msg->bus == 2U) {
    if (msg->addr == FCA_GIORGIO_ACC_2) {
      // When using stock ACC, enter controls on rising edge of stock ACC engage, exit on disengage
      // Signal: ACC_2.CRUISE_STATUS
      bool cruise_engaged = GET_BIT(msg, 23U);
      acc_main_on = true;
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool fca_giorgio_tx_hook(const CANPacket_t *msg) {
  // lateral limits
  const TorqueSteeringLimits FCA_GIORGIO_STEERING_LIMITS = {
    .max_torque = 600,
    .max_rt_delta = 250,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .driver_torque_allowance = 80,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  // Safety check for commanded steering torque
  if (msg->addr == FCA_GIORGIO_LKA_COMMAND) {
    // Signal: LKA_COMMAND.LKA_TORQUE
    int desired_torque = ((msg->data[0] << 3) | (msg->data[1] >> 5)) - 1024U;
    // Signal: LKA_COMMAND.LKA_ACTIVE
    bool steer_req = GET_BIT(msg, 12U);

    if (steer_torque_cmd_checks(desired_torque, steer_req, FCA_GIORGIO_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // TODO: sanity check cancel spam, once a button message is found

  return tx;
}

const safety_hooks fca_giorgio_hooks = {
  .init = fca_giorgio_init,
  .rx = fca_giorgio_rx_hook,
  .tx = fca_giorgio_tx_hook,
  .get_counter = fca_giorgio_get_counter,
  .get_checksum = fca_giorgio_get_checksum,
  .compute_checksum = fca_giorgio_compute_crc,
};
