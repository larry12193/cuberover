/*******************************************************************************
 * canopen.cpp - CANopen base source file
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 ******************************************************************************/

#include "canopen.h"

// Constructor
canopen::canopen(std::string socket_name, ros::NodeHandle *nh) {
    _ifname = socket_name;
    _bus_pub = nh->advertise<tmcm_1633_driver::canMsg>("/can_messages",1000);
}

// Destructor
canopen::~canopen() {
    close(_s);
}

/*  @brief Initializes the socket for can communications

*/
int8_t canopen::init() {
    // Close already open socket, if exists
    if( _s > 0 ) {
        close(_s);
    }

    // Attempt to open socket
    if((_s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket.");
        return -1;
    }

    // Get can network interface index number
    strcpy(_ifr.ifr_name, _ifname.c_str());
    ioctl(_s, SIOCGIFINDEX, &_ifr);

    _addr.can_family  = AF_CAN;
    _addr.can_ifindex = _ifr.ifr_ifindex;

    // Display found index number
    printf("%s at index %d\n", _ifname.c_str(), _ifr.ifr_ifindex);

    // Attempt to bind socket
    if(bind(_s, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
        perror("Error in socket bind.");
        return -2;
    }

    memset(_fds, 0, sizeof(_fds));
    _fds[0].fd = _s;
    _fds[0].events = POLLIN;
    _nfds = 1;
    _timeout = 10;
    return 0;
}

/*  @brief Writes data to object in dictionary via SDO

*/
int8_t canopen::sdo_write(uint8_t nodeID, co_obj_t reg, uint8_t *data) {
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
            break;
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
    _nbytes = write(_s, &_frame, sizeof(struct can_frame));
    return 0;
}

/*  @brief Reads data from object in dictionary via SDO

*/
int8_t canopen::sdo_read(uint8_t nodeID, co_obj_t reg) {
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
            break;
        default:
            perror("Invalid SDO data length.");
            return -1;
    }
    // Fill in object index and subindex
    _frame.data[1] = (uint8_t)(reg.index & 0x00FF);
    _frame.data[2] = (uint8_t)(reg.index >> 8);
    _frame.data[3] = reg.subindex;

    // Write data
    _nbytes = write(_s, &_frame, sizeof(struct can_frame));
    return 0;
}

/*  @brief Command NMT state change

*/
int8_t canopen::send_nmt(uint8_t nodeID, uint8_t mode) {
    // Fill CAN frame with pertinent data
    _frame.can_id  = NMT_CONTROL; // Set can ID to NMT control
    _frame.can_dlc = 2;           // Set data length to message data size
    _frame.data[0] = mode;
    _frame.data[1] = nodeID;

    // Write data
    _nbytes = write(_s, &_frame, sizeof(struct can_frame));
    return 0;
}

/* @brief Reads data from canbus and publishes anything found

*/
void canopen::read_bus(const ros::TimerEvent& e) {
    _rc = poll(_fds, _nfds, _timeout);

    if( _rc > 0 ) {
        //ROS_INFO("reading bus, count = %d",_nfds);
        _nbytes = read(_s, &_frame, sizeof(struct can_frame));
        if( _nbytes == sizeof(struct can_frame)) {
            // Fill in time stamp
            _canmsg.header.stamp = ros::Time::now();
            // Extract COB ID
            _canmsg.cobID = _frame.can_id & 0xFF0;
            // Extract node ID
            _canmsg.nodeID = _frame.can_id & 0x00F;
            // Extract data, if there is any
            if( _frame.can_dlc > 0 ) {
                switch(_frame.data[0] & 0x0F) {
                    case SDO_1_BYTE:
                        _canmsg.dlc = 1;
                        break;
                    case SDO_2_BYTE:
                        _canmsg.dlc = 2;
                        break;
                    case SDO_4_BYTE:
                        _canmsg.dlc = 4;
                        break;
                }
                // Extract message register identifier
                _canmsg.reg = ((uint16_t)_frame.data[2] << 8) || (uint16_t)_frame.data[1];
                // Extract data
                _canmsg.data.clear();
                for(int i = 0; i < _canmsg.dlc; i++) {
                    _canmsg.data.push_back(_frame.data[4+i]);
                }
            } else {
                // No data in message
                _canmsg.dlc = 0;
                _canmsg.data.clear();
            }
            // Publish message
            _bus_pub.publish(_canmsg);
        }
    }
}
