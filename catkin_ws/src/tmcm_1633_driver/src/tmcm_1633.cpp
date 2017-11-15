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
tmcm_1633::tmcm_1633(uint8_t nodeID, canopen *canbase, ros::NodeHandle *nh) {
  _nodeID = nodeID;
  _canbase = canbase;

  _can_sub = nh->subscribe<tmcm_1633_driver::canMsg>("/can_messages",100, &tmcm_1633::process_message, this);
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

/* @brief Returns whether or not the motor is ready after reset

*/
uint8_t tmcm_1633::ready() {
    return _nmt_state != NMT_BOOT;
}

/*  @brief Sets the mode of operation of the motor driver
      1: Position mode
      3: Velocity mode
     10: Cyclic torque mode

*/
int8_t tmcm_1633::set_mode(uint8_t mode) {

    // Make sure the node is in operational mode
    if(_nmt_state != NMT_OPERATE ) {
        if( _canbase->send_nmt(_nodeID, NMT_CMD_OP) < 0 ) {
            perror("Unable to set node to operational");
            return -2;
        }
    }

    // Initialize depending on which mode
    switch(mode) {
        // Velocity mode
        case VEL_MODE:
            // Fill in mode data in can bus message data
            data[0] = mode;
            if( _canbase->sdo_write(_nodeID, OPERATION_MODE_OBJ, data) < 0) {
                perror("Error changing mode via SDO");
                return -1;
            }
            ros::spinOnce();
            // Reset control sequence
            data[0] = CONTROL_SHUTDOWN;
            data[1] = 0x00;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to reset control sequence.");
                return -1;
            }
            ros::spinOnce();
            ros::Duration(0.1).sleep();
            // Set control sequence to switched_on state
            data[0] = CONTROL_SWITCH_ON;
            data[1] = 0x00;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to set to switched on state.");
                return -1;
            }
            ros::spinOnce();
            // Initialize velocity as zero rpm
            data[0] = 0x00;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            if( _canbase->sdo_write(_nodeID, TARG_VEL_OBJ, data) < 0) {
                perror("Unable to set to initial velocity.");
                return -1;
            }
            ros::spinOnce();
            // Set control sequence to operation_enabled
            data[0] = CONTROL_OP_ENABLE;
            data[1] = 0x00;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to set to operation enabled state.");
                return -1;
            }
            ros::spinOnce();
            break;

        // Torque mode
        case TOR_MODE:
            // Disable limit switch inputs
            data[0] = 0x03;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            if( _canbase->sdo_write(_nodeID, LIMIT_SWITCH_OBJ, data) < 0) {
                perror("Unable to disable limit switch inputs.");
                return -1;
            }
            // Fill in mode data in can bus message data
            data[0] = mode;
            if( _canbase->sdo_write(_nodeID, OPERATION_MODE_OBJ, data) < 0) {
                perror("Error changing mode via SDO");
                return -1;
            }
            // Reset control sequence
            data[0] = CONTROL_SHUTDOWN;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to reset control sequence.");
                return -1;
            }
            // Set control sequence to switched_on state
            data[0] = CONTROL_SWITCH_ON;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to set to switched on state.");
                return -1;
            }
            // Initialize torque to 0 mA
            data[0] = 0x00;
            data[1] = 0x00;
            data[2] = 0x00;
            data[3] = 0x00;
            if( _canbase->sdo_write(_nodeID, TARG_TORQUE_OBJ, data) < 0) {
                perror("Unable to set initial torque.");
                return -1;
            }
            // Set control sequence to operation_enabled
            data[0] = CONTROL_OP_ENABLE;
            if( _canbase->sdo_write(_nodeID, CONTROL_WORD_OBJ, data) < 0) {
                perror("Unable to set to operation enabled state.");
                return -1;
            }
            break;
        default:
            ROS_WARN("Unknown controller state requested!");
            break;
    }
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
void tmcm_1633::process_message(const tmcm_1633_driver::canMsg::ConstPtr& msg) {
    if( msg->nodeID == _nodeID ) {
        // Process by COB ID
        switch(msg->cobID) {

          // Driver responsed to NMT reset
          case NMT_RESPONSE:
            _nmt_state = NMT_PREOP;
            break;

          // Driver responds from SDO read request
          case SDO_TRANSMIT:
            if( msg->reg == ACTUAL_POS_OBJ.index ) {
              pos = msg->data[0] || (msg->data[1] << 8) || (msg->data[2] << 16) || (msg->data[3] << 24);
            }
            break;
        }
    }
}
