/*******************************************************************************
 * see3cam_cu51_driver.cpp - ROS driver for See3CAM_CU51 12-bit monochrome
 *                           camera
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 * Adapted from econ-Systems OpenCV examples found here,
 * https://www.e-consystems.com/blog/camera/accessing-see3cam-custom-format-with-opencv/
 *
 ******************************************************************************/
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "uvccamera.h"

#include <iostream>
#include <unistd.h>
#include <termios.h>

using namespace cv;
using namespace std;

uint16_t exposure;
uint8_t  brightness;

void setExposureCallback(const std_msgs::UInt16::ConstPtr& msg) {
  exposure = msg->data;
}

void setBrightnessCallback(const std_msgs::UInt8::ConstPtr& msg) {
  brightness = msg->data;
}

int main (int argc, char **argv) {

  ros::init(argc, argv, "see3cam_cu51_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  int id, initExposure, initBrightness;
  double width, height;
  std::string portName;

  priv_nh.param<int>("camera_id", id,     1);
  priv_nh.param<int>("start_exp", initExposure, 5);
  priv_nh.param<int>("start_bri", initBrightness, 10);
  priv_nh.param<double>("height", height, 1944);
  priv_nh.param<double>("width",  width,  2592);
  priv_nh.param<std::string>("hid_port", portName, "/dev/see3cam_cu51_hidraw");

	VideoCapture _CameraDevice;
	Mat ResultImage, InputImage;

  uvccamera uvc(portName);

  // Initialize UVC connection
  bool ret = uvc.initCamera();
  if( !ret ) {
    ROS_ERROR("Unable to gain access to UVC control through %s", portName.c_str());
  } else {
    ret = uvc.setExposure(initExposure);
    if( !ret ) {
      ROS_ERROR("Failed to set initial exposure");
    } else {
      exposure = initExposure;
    }
  }

  ROS_INFO("Attempting to open camera at ID %d with %0.0fx%0.0f resolution",id,width,height);
	//Open the device at the ID 0
	_CameraDevice.open(id);
	if( !_CameraDevice.isOpened()) //Check for the device
	{
		ROS_ERROR("No camera found with ID %d", id);
		return -1;
	}

	//Set up the width and height of the camera
	_CameraDevice.set(CV_CAP_PROP_FRAME_WIDTH,  width);
	_CameraDevice.set(CV_CAP_PROP_FRAME_HEIGHT, height);
  _CameraDevice.set(CV_CAP_PROP_BRIGHTNESS, (float)initBrightness/100);

  // Define image message
  cv_bridge::CvImage out_msg;

  // Initialize publisher
  ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("/camera/image", 100);

  // Initialize subscriber
  ros::Subscriber exp_sub = nh.subscribe<std_msgs::UInt16>("/set_exposure", 1000, setExposureCallback);
  ros::Subscriber bri_sub = nh.subscribe<std_msgs::UInt8>("/set_brightness", 1000, setBrightnessCallback);
  // Set loop rate
  ros::Rate loop_rate(10);

	while(ros::ok())
	{

    if( exposure != initExposure ) {
      ret = uvc.setExposure(exposure);
      if(!ret) {
        ROS_WARN("Failed to set exposure!");
      } else {
        initExposure = exposure;
      }
    }

    if( brightness != initBrightness ) {
      _CameraDevice.set(CV_CAP_PROP_BRIGHTNESS, (float)brightness/100);
      initBrightness = brightness;
    }

    _CameraDevice >> InputImage; //Read the input image

		if(!InputImage.empty()) //Check for the vlid image
		{
      //Convert to 8 Bit:
                  //Scale the 12 Bit (4096) Pixels into 8 Bit(255) (255/4096)= 0.06226
  		convertScaleAbs(InputImage, ResultImage, 0.06226);


      out_msg.header.stamp = ros::Time::now(); // Same timestamp and tf frame as input image
      out_msg.encoding     = sensor_msgs::image_encodings::MONO8; // Or whatever
      out_msg.image        = ResultImage; // Your cv::Mat

      img_pub.publish(out_msg.toImageMsg());
		}
    ros::spinOnce();
    loop_rate.sleep();
	}

	//Release the devices
	_CameraDevice.release();

	return 1;
}
