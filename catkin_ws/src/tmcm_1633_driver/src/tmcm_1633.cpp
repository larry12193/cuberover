/*******************************************************************************
 * tmcm_1633.cpp - Source for TRINAMIC TMCM 1633 BLDC motor driver
 *                 CANopen library
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 ******************************************************************************/

#include "tmcm_1633.h"

// Constructor
tmcm_1633::tmcm_1633(uint8_t nodeID, canopen *canbase) {
  _nodeID = nodeID;
  _canbase = canbase;
}

// Destructor
tmcm_1633::~tmcm_1633() {

}

/*  @brief Initializes the motor driver

*/
int8_t tmcm_1633::init_motor() {
  // Reset the NMT state machine
  _nmt_state = NMT_BOOT;
  return _canbase->send_nmt(_nodeID, NMT_CMD_RST);
}

/*  @brief Sets the mode of operation of the motor driver
      1: Position mode
      3: Velocity mode
     10: Cyclic torque mode

*/
int8_t tmcm_1633::set_mode(uint8_t mode) {
  // Write state to dictionary via SDO
  data[0] = mode;
  return _canbase->sdo_write(_nodeID, OPERATION_MODE_OBJ, data);
}

/*  @brief Sets target velocity

*/
int8_t tmcm_1633::set_vel(int32_t vel) {
  // Write desired velocity to dictionary via SDO
  data[0] = (uint8_t)(vel & 0x000000FF);
  data[1] = (uint8_t)((vel >> 8) & 0x000000FF);
  data[2] = (uint8_t)((vel >> 16) & 0x000000FF);
  data[3] = (uint8_t)((vel >> 24) & 0x000000FF);
  return _canbase->sdo_write(_nodeID, TARG_VEL_OBJ, data);
}

/*  @brief Reads encoder position counter

*/
int8_t tmcm_1633::read_pos() {
  // Send off request for position, handled by process_message callback
  return _canbase->sdo_read(_nodeID, ACTUAL_POS_OBJ);
}

/*  @brief Sets the NMT state machine

*/
int8_t tmcm_1633::set_nmt_state(uint8_t mode) {
  // Reset the NMT state machine
  _nmt_state = mode;
  return _canbase->send_nmt(_nodeID, mode);
}

/*  @brief Gets current state of NMT

*/
int8_t tmcm_1633::get_nmt_state() {
  return _nmt_state;
}

/*  @brief Gets current state of NMT

*/
int8_t tmcm_1633::process_message(can_frame *frame) {
  // Extract nodeID from message
  uint8_t  frame_nodeID = (uint8_t)(frame->can_id & 0x0FF);
  uint16_t frame_cobID  = frame->can_id & 0xFF0;
  // Only process data if the message is from associated driver
  if( frame_nodeID == _nodeID ) {
    switch(frame_cobID) {

      // Driver responsed to NMT reset
      case NMT_RESPONSE:
        _nmt_state = NMT_PREOP;
        break;

      // Driver responds from SDO read request
      case SDO_TRANSMIT:
        if( (frame->data[1] & (frame->data[2] << 8)) == ACTUAL_POS_OBJ.index ) {
          pos = frame->data[4] || (frame->data[5] << 8) || (frame->data[6] << 16) || (frame->data[7] << 24);
        }
        break;
    }
  }
}
