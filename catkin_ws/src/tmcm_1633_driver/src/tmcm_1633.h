/*******************************************************************************
 * tmcm_1633.h - Header for TRINAMIC TMCM 1633 BLDC motor driver
 *               CANopen library
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 ******************************************************************************/

#ifndef TMCM_1633_H
#define TMCM_1633_H

#include "canopen.h"

#define POS_MODE           0x01
#define VEL_MODE           0x03
#define TOR_MODE           0x0A

#define CONTROL_SHUTDOWN   0x06
#define CONTROL_SWITCH_ON  0x07
#define CONTROL_OP_ENABLE  0x0F

#define OVERCURRENT_FLAG   0
#define UNDERVOLTAGE_FLAG  1
#define OVERVOLTAGE_FLAG   2
#define OVERTEMP_FLAG      3
#define MOTOR_HALTED       4
#define HALL_ERROR         5
#define DRIVER_ERROR       6
#define INIT_ERROR         7
#define STOP_MODE          8
#define VELOCITY_MODE      9
#define POSITION_MODE      10
#define TORQUE_MODE        11
#define EMER_STOP          12
#define FREE_RUNNING       13
#define POSITION_END       14
#define MODULE_INIT        15
#define IIT_EXCEEDED       17

#define SAVE_ALL 0x65766173  // Saves all current parameters to EEPROM

// Store Parameters
const co_obj_t STORE_PARAM_OBJ = {
  .index = 0x1010,
  .subindex = 0x01,
  .size = 4
};

// Limit Switches
const co_obj_t LIMIT_SWITCH_OBJ = {
    .index = 0x2005,
    .subindex = 0x00,
    .size = 4
};

// Status Flags
const co_obj_t STATUS_FLAG_OBJ = {
  .index = 0x200D,
  .subindex = 0x00,
  .size = 4
};

// Motor Poles
const co_obj_t MOTOR_POLES_OBJ = {
  .index = 0x2010,
  .subindex = 0x00,
  .size = 1
};

// Limits - Maximum torque
const co_obj_t LIMITS_TOR_OBJ = {
  .index = 0x2020,
  .subindex = 0x01,
  .size = 4
};

// Limits - Maximum Velocity
const co_obj_t LIMITS_VEL_OBJ = {
  .index = 0x2020,
  .subindex = 0x02,
  .size = 4
};

// Limits - Maximum Acceleration
const co_obj_t LIMITS_ACC_OBJ = {
  .index = 0x2020,
  .subindex = 0x03,
  .size = 4
};

// Commutation Mode (6: hall sensor, 7: encoder, 8: controlled)
const co_obj_t COMMUTE_OBJ = {
  .index = 0x2055,
  .subindex = 0x00,
  .size = 1
};

// Hall Sensor Settings - Inversion
const co_obj_t HALL_INV_OBJ = {
  .index = 0x2070,
  .subindex = 0x02,
  .size = 1
};

// ABN Encoder Settings - Direction
const co_obj_t ENCODER_DIR_OBJ = {
  .index = 0x2080,
  .subindex = 0x04,
  .size = 1
};

// Control Word
const co_obj_t CONTROL_WORD_OBJ = {
  .index = 0x6040,
  .subindex = 0x00,
  .size = 2
};

// Mode of Operation ( 1: position, 3: velocity, 10: torque)
const co_obj_t OPERATION_MODE_OBJ = {
  .index = 0x6060,
  .subindex = 0x00,
  .size = 1
};

// Position Actual Value
const co_obj_t ACTUAL_POS_OBJ = {
  .index = 6063,
  .subindex = 0x00,
  .size = 4
};

// Target Velocity
const co_obj_t TARG_VEL_OBJ = {
  .index = 0x60FF,
  .subindex = 0x00,
  .size = 4
};

// Target torque
const co_obj_t TARG_TORQUE_OBJ = {
  .index = 0x6071,
  .subindex = 0x00,
  .size = 4
};

class tmcm_1633 {
private:
  uint8_t _nodeID;
  uint8_t _nmt_state;

  uint8_t data[4];

  canopen * _canbase;

  ros::Subscriber _can_sub;

public:
  tmcm_1633(uint8_t nodeID, canopen *canbase, ros::NodeHandle *nh);
  ~tmcm_1633();

  int32_t pos;

  int8_t init_motor();
  uint8_t ready();
  int8_t set_mode(uint8_t mode);
  int8_t set_vel(int32_t vel);
  int8_t read_pos();
  int8_t set_nmt_state(uint8_t state);
  int8_t get_nmt_state();
  void process_message(const tmcm_1633_driver::canMsg::ConstPtr& msg);
};

#endif // TMCM_1633_H
