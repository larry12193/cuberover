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

int main(int argc, char **argv) {

 ros::init(argc, argv, "see3cam_cu51_node");
 ros::NodeHandle nh;
 ros::NodeHandle priv_nh("~");

 int s;
 int nbytes;
 struct sockaddr_can addr;
 struct can_frame frame;
 struct ifreq ifr;

 const char *ifname = "can0";

 if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
   perror("Error while opening socket");
   return -1;
 }

 strcpy(ifr.ifr_name, ifname);
 ioctl(s, SIOCGIFINDEX, &ifr);

 addr.can_family  = AF_CAN;
 addr.can_ifindex = ifr.ifr_ifindex;

 printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

 if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
   perror("Error in socket bind");
   return -2;
 }

 ros::Rate lrate(10);

 // NMT Reset signal
 frame.can_id  = 0x000;
 frame.can_dlc = 2;
 frame.data[0] = 0x81;
 frame.data[1] = 0x00;
 ROS_INFO("Sending NMT reset...");
 nbytes = write(s, &frame, sizeof(struct can_frame));

 bool reset = false;
 while(!reset) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   ROS_INFO("Waiting for heartbeat...");
   if( nbytes == sizeof(struct can_frame)) {
     if( frame.can_id == 0x701 ) {
       reset = true;
       ROS_INFO("Device reset!");
     }
     ros::spinOnce();
     lrate.sleep();
   }
 }
 ros::Duration(1).sleep();

 int i;
 bool rec;

 // Set velocity mode
 frame.can_id  = 0x601;
 frame.can_dlc = 5;
 frame.data[0] = 0x2F;
 frame.data[1] = 0x60;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x03;
 ROS_INFO("Entering velocity mode");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }



 // Set encoder commutation mode
 frame.can_id  = 0x601;
 frame.can_dlc = 5;
 frame.data[0] = 0x2F;
 frame.data[1] = 0x55;
 frame.data[2] = 0x20;
 frame.data[3] = 0x00;
 frame.data[4] = 0x07;
 ROS_INFO("Set encoder commutation mode");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }

 // Ready to switch on state
 frame.can_id  = 0x601;
 frame.can_dlc = 6;
 frame.data[0] = 0x2B;
 frame.data[1] = 0x40;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x06;
 frame.data[5] = 0x00;
 ROS_INFO("Ready to switch on");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }

 // Switch on state
 frame.can_id  = 0x601;
 frame.can_dlc = 6;
 frame.data[0] = 0x2B;
 frame.data[1] = 0x40;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x07;
 frame.data[5] = 0x00;
 ROS_INFO("Switched on");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }

 // Set target velocity (1000 rpm)
 frame.can_id  = 0x601;
 frame.can_dlc = 8;
 frame.data[0] = 0x23;
 frame.data[1] = 0xFF;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x3E;
 frame.data[5] = 0x08;
 frame.data[6] = 0x00;
 frame.data[7] = 0x00;
 ROS_INFO("Setting velocity to 1000rpm");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }

 // Enable operation
 frame.can_id  = 0x601;
 frame.can_dlc = 5;
 frame.data[0] = 0x2B;
 frame.data[1] = 0x40;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x0F;
 frame.data[5] = 0x00;
 ROS_INFO("Enabled Operation");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }
 ros::Duration(10).sleep();

 // Set target velocity (0 rpm)
 frame.can_id  = 0x601;
 frame.can_dlc = 8;
 frame.data[0] = 0x23;
 frame.data[1] = 0xFF;
 frame.data[2] = 0x60;
 frame.data[3] = 0x00;
 frame.data[4] = 0x00;
 frame.data[5] = 0x00;
 frame.data[6] = 0x00;
 frame.data[7] = 0x00;
 ROS_INFO("Setting velocity to 0rpm");
 nbytes = write(s, &frame, sizeof(struct can_frame));
 rec = false;
 while(ros::ok() && !rec) {
   nbytes = read(s, &frame, sizeof(struct can_frame));
   if( nbytes == sizeof(struct can_frame)) {
     ROS_INFO("Recived message from ID: %04x", frame.can_id);
     for( i = 0; i < frame.can_dlc; i++ ) {
       ROS_INFO("Data%d: %04x",i,frame.data[i]);
     }
     rec = true;
   }
   ros::spinOnce();
   lrate.sleep();
 }

 close(s);

 // int i;
 // while(ros::ok()) {
 //   nbytes = read(s, &frame, sizeof(struct can_frame));
 //   if( nbytes == sizeof(struct can_frame)) {
 //     ROS_INFO("Recived message from ID: %04x", frame.can_id);
 //     for( i = 0; i < frame.can_dlc; i++ ) {
 //       ROS_INFO("Data%d: %04x",i,frame.data[i]);
 //     }
 //   }
 //
 //   ros::spinOnce();
 //   lrate.sleep();
 // }

 return 0;
}
