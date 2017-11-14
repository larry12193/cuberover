/*******************************************************************************
 * canopen.cpp - CANopen base source file
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 ******************************************************************************/

#include "canopen.h"

// Constructor
canopen::canopen(sting socket_name) {
  _ifname = socket_name;
}

// Destructor
canopen::~canopen() {

}

/*  @brief Initializes the socket for can communications

*/
int8_t canopen::init() {
  // Attempt to open socket
  if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
    perror("Error while opening socket.");
    return -1;
  }

  // Get can network interface index number
  _ifr.ifr_name = _ifname.c_str();
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex

  // Display found index number
  printf("%s at index %d\n", _ifname, _ifr.ifr_ifindex);

  // Attempt to bind socket
  if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind.");
    return -2;
  }
  return 0;
}

/*  @brief Writes data to object in dictionary via SDO

*/
int8_t canopen::sdo_write(uin8_t nodeID, co_obj_t reg, uint8_t *data) {
  // Fill CAN frame with pertinent data
  _frame.can_id  = SDO_RECEIVE + nodeID; // Set can ID to SDO COB and nodeID
  _frame.can_dlc = reg.size + 4;        // Set data length to message data size plus command/reg bytes
  // Fill in the proper data size
  switch(reg.size) {
    case 4:
      _frame.data[0] = SDO_WRITE + SDO_4_BYTE;
      break;
    case 2:
      _frame.data[0] = SDO_WRITE + SDO_2_BYTE;
      break;
    case 1:
      _frame.data[0] = SDO_WRITE + SDO_1_BYTE;
      break
    default:
      perror("Invalid SDO data length.");
      return -1;
  }
  // Fill in object index and subindex
  _frame.data[1] = (uint8_t)(reg.index & 0x00FF);
  _frame.data[2] = (uint8_t)(reg.index >> 8);
  _frame.data[3] = reg.subindex;

  // Fill in data
  for( uint8_t i = 0; i < reg.size; i++ ) {
    _frame.data[4+i] = data[i];
  }

  // Write data
  nbytes = write(_s, &_frame, sizeof(struct can_frame));
  return 0;
}

/*  @brief Reads data from object in dictionary via SDO

*/
int8_t canopen::sdo_read(uin8_t nodeID, co_obj_t reg) {
  // Fill CAN frame with pertinent data
  _frame.can_id  = SDO_RECEIVE + nodeID; // Set can ID to SDO write and nodeID
  _frame.can_dlc = reg.size + 4;        // Set data length to message data size plus command/reg bytes
  // Fill in the proper data size
  switch(reg.size) {
    case 4:
      _frame.data[0] = SDO_READ + SDO_4_BYTE;
      break;
    case 2:
      _frame.data[0] = SDO_READ + SDO_2_BYTE;
      break;
    case 1:
      _frame.data[0] = SDO_READ + SDO_1_BYTE;
      break
    default:
      perror("Invalid SDO data length.");
      return -1;
  }
  // Fill in object index and subindex
  _frame.data[1] = (uint8_t)(reg.index & 0x00FF);
  _frame.data[2] = (uint8_t)(reg.index >> 8);
  _frame.data[3] = reg.subindex;

  // Write data
  nbytes = write(_s, &_frame, sizeof(struct can_frame));
  return 0;
}

/*  @brief Command NMT state change

*/
int8_t canopen::send_nmt(uint8_t nodeID, uin8_t mode) {
  // Fill CAN frame with pertinent data
  _frame.can_id  = NMT_CONTROL; // Set can ID to NMT control
  _frame.can_dlc = 2;           // Set data length to message data size
  _frame.data[0] = mode;
  _frame.data[1] = nodeID;

  // Write data
  nbytes = write(_s, &_frame, sizeof(struct can_frame));
  return 0;
}
