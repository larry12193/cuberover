/*******************************************************************************
 * tmcm_1633_driver.cpp - ROS driver for TRINAMIC TMCM 1633 BLDC motor driver
 *                        via CANOpen interface
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 *
 ******************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "canopen.h"
#include "tmcm_1633.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "see3cam_cu51_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    // Define canopen object and initialize
    canopen canbase("can0", &nh);
    if( canbase.init() < 0 ) {
        perror("Unable to configure canbus. Check socket configuration.");
        return 0;
    }

    // Create timer callback that initiates can bus monitoring
    ros::Timer can_read = nh.createTimer(ros::Duration(0.1), &canopen::read_bus, &canbase);

    // Create and initialize motor driver objects
    tmcm_1633 motorR(0x01, &canbase);
    tmcm_1633 motorL(0x02, &canbase);

    // Send NMT reset to all nodes
    canbase.send_nmt(NMT_ALL_NODE, NMT_CMD_RST);

    ros::spin();

    return 0;
}
