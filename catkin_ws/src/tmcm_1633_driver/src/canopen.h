/*******************************************************************************
 * canopen.h - CANopen base header file
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 ******************************************************************************/

#ifndef CANOPEN_H
#define CANOPEN_H

#include "ros/ros.h"
#include "tmcm_1633_driver/canMsg.h"

#include <stdio.h>
#include <stdlib.h>
#include <poll.h>
#include <unistd.h>
#include <string>
#include <net/if.h>
#include <inttypes.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#define SDO_TRANSMIT  0x580  // SDO transmit (from device) base COB ID
#define SDO_RECEIVE   0x600  // SDO recieve (to device) base COB ID

#define NMT_RESPONSE  0x700  // NMT response (from device) base COB ID
#define NMT_CONTROL   0x000  // NMT control (to device) base COB ID

#define SYNC          0x080  // Sync message COB ID

#define TPDO_1        0x180  // TPDO 1 COB ID
#define TPDO_2        0x280  // TPDO 2 COB ID
#define TPDO_3        0x380  // TPDO 3 COB ID
#define TPDO_4        0x480  // TPDO 4 COB ID
#define RPDO_1        0x200  // RPDO 1 COB ID
#define RPDO_2        0x300  // RPDO 2 COB ID
#define RPDO_3        0x400  // RPDO 3 COB ID
#define RPDO_4        0x500  // RPDO 4 COB ID

#define SDO_1_BYTE    0x0F   // Indicates message data length of 1 byte
#define SDO_2_BYTE    0x0B   // Indicates message data length of 2 bytes
#define SDO_4_BYTE    0x03   // Indicates message data length of 4 bytess

#define SDO_WRITE     0x20   // SDO write identifier
#define SDO_READ      0x40   // SDO read identifier

#define NMT_BOOT      0x00
#define NMT_STOPPED   0x04
#define NMT_OPERATE   0x05
#define NMT_PREOP     0x7F
#define NMT_ALL_NODE  0x00

#define NMT_CMD_OP    0x01   // NMT enter operational command
#define NMT_CMD_PREOP 0x80   // NMT enter preoperational command
#define NMT_CMD_STOP  0x02   // NMT enter stop command
#define NMT_CMD_RST   0x81   // NMT reset command
#define NMT_CMD_RSTC  0x82   // NMT reset communications command

#define NMT_RST_TIMEOUT 5

typedef struct co_obj_t {
  uint16_t index;     // SDO index
  uint16_t subindex;  // SDO subindex
  uint8_t  size;      // SDO data size (bytess)
} co_obj_t;

class canopen {

private:
  struct sockaddr_can _addr;
  struct can_frame _frame;
  struct ifreq _ifr;
  uint8_t _s;
  uint8_t _nbytes;
  std::string _ifname;
  struct pollfd _fds[200];
  int _nfds;
  int _timeout;
  int _rc;


  ros::Publisher _bus_pub;
  tmcm_1633_driver::canMsg _canmsg;

public:
  canopen(std::string socket_name, ros::NodeHandle *nh);
  ~canopen();

  int8_t init();
  int8_t sdo_write(uint8_t nodeID, co_obj_t reg, uint8_t *data);
  int8_t sdo_read(uint8_t nodeID, co_obj_t reg);
  int8_t send_nmt(uint8_t nodeID, uint8_t mode);
  void read_bus(const ros::TimerEvent& e);
};

#endif // CANOPEN_H
