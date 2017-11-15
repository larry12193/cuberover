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
#include "sensor_msgs/Joy.h"

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

bool resetNodes = false;
bool changeVel = false;
int32_t leftVel = 0;
int32_t rightVel = 0;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // B button means reset nodes
    if( msg->buttons[1] ) {
        resetNodes = true;
    }

    if( std::fabs(msg->axes[1]) > 0.1 ) {
        changeVel = true;
        ROS_INFO("Setting velocity to  %X, from %f",(int32_t)(msg->axes[1]*1000.0),msg->axes[1]);
        leftVel = (int32_t)(msg->axes[1]*1000.0);
        rightVel = leftVel;
    } else {
        changeVel = true;
        ROS_INFO("Setting velocity to  %d",(int32_t)(msg->axes[1]*1000.0));
        leftVel = 0;
        rightVel = leftVel;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tmcm_1633_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy",5,joyCallback);

    ros::Rate loop_rate(100);

    ros::Time startTime;

    // Define canopen object and initialize
    canopen canbase("can0", &nh);
    if( canbase.init() < 0 ) {
        perror("Unable to configure canbus. Check socket configuration.");
        return 0;
    }

    // Create timer callback that initiates can bus monitoring
    ros::Timer can_read = nh.createTimer(ros::Duration(0.1), &canopen::read_bus, &canbase);

    // Create and initialize motor driver objects
    tmcm_1633 motorR(0x01, &canbase, &nh);
    tmcm_1633 motorL(0x02, &canbase, &nh);

    // Initialize drivers
    motorR.init_motor();
    motorL.init_motor();

    // Wait for motor drivers to respond from reset
    startTime = ros::Time::now();
    while( !motorR.ready() && !motorL.ready()) {
        loop_rate.sleep();
        ros::spinOnce();

        // Check for timeout
        if( ros::Time::now() - startTime > ros::Duration(NMT_RST_TIMEOUT) ) {
            // Notify which driver(s) is the problem
            if( !motorR.ready() ) {
                ROS_ERROR("No response from %s motor driver after NMT reset.", "left");
            }
            if( !motorL.ready() ) {
                ROS_ERROR("No response from %s motor driver after NMT reset.", "right");
            }
            ROS_INFO("Attempting reset");
            system("ip link set down can0");
            system("ip link set up can0");
            startTime = ros::Time::now();
        }
    }
    ROS_INFO("Motors reset.");

    // Set both motors to velocity mode
    motorR.set_mode(VEL_MODE);
    motorL.set_mode(VEL_MODE);
    ROS_INFO("Entered velocity mode...");

    while(ros::ok()) {

        if( resetNodes ) {
            ROS_INFO("Reseting drives...");
            resetNodes = false;
            // Initialize drivers
            motorR.init_motor();
            motorL.init_motor();

            // Wait for motor drivers to respond from reset
            startTime = ros::Time::now();
            while( !motorR.ready() && !motorL.ready()) {
                loop_rate.sleep();
                ros::spinOnce();

                // Check for timeout
                if( ros::Time::now() - startTime > ros::Duration(NMT_RST_TIMEOUT) ) {
                    // Notify which driver(s) is the problem
                    if( !motorR.ready() ) {
                        ROS_ERROR("No response from %s motor driver after NMT reset.", "left");
                    }
                    if( !motorL.ready() ) {
                        ROS_ERROR("No response from %s motor driver after NMT reset.", "right");
                    }
                    ROS_INFO("Attempting reset");
                    system("sudo ip link set down can0");
                    system("sudo ip link set up can0");
                    startTime = ros::Time::now();
                    // Initialize drivers
                    motorR.init_motor();
                    motorL.init_motor();
                }
            }

            // Set both motors to velocity mode
            motorR.set_mode(VEL_MODE);
            motorL.set_mode(VEL_MODE);
            ROS_INFO("Drives reset");
        } else if( changeVel ) {
            changeVel = false;
            motorR.set_vel(rightVel);
            motorL.set_vel(leftVel);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
