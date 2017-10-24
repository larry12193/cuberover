/*******************************************************************************
 * uvccamera.h - UVC camera interface for see3cam_cu51 camera from econ
 *               systems
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 * Adapted from econ Systems QtCam viewer application
 * https://github.com/econsysqtcam/qtcam
 ******************************************************************************/

#ifndef UVCCAMERA_H
#define UVCCAMERA_H

#include <stdlib.h>
#include <string>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>
#include <inttypes.h>
#include <string.h>
#include <fcntl.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#define APPLICATION_READY 	0x12
#define READFIRMWAREVERSION	0x40
#define GETCAMERA_UNIQUEID	0x41
#define ENABLEMASTERMODE	0x50
#define ENABLETRIGGERMODE	0x51
#define CAMERA_CONTROL_51	0x65

#define GET_EXPOSURE_VALUE	0x01
#define SET_EXPOSURE_VALUE	0x02
#define TRIGGER_STILL_CAPTURE	0x03
#define GET_TORCH_LEVEL_51	0x04
#define SET_TORCH_LEVEL_51	0x05

#define EXP_FAIL		        0x00
#define EXP_SUCCESS		      0x01

#define TRIGGER_FAIL		    0x00
#define TRIGGER_SUCCESS	  	0x01

#define BUFFER_LENGTH       65
#define TIMEOUT             2000

class uvccamera
{
public:
  uvccamera(std::string port);
  ~uvccamera();

  bool initCamera();
  bool enableMasterMode();
  bool setExposure(uint16_t newValue);
  bool getExposure();
  // bool setBrightness();
  // bool getBrightness();
  unsigned int getTickCount();

  uint16_t curExposure;
  uint8_t  curBrightness;

private:
  std::string _port;
  unsigned char _out_buf[BUFFER_LENGTH];
  unsigned char _in_buf[BUFFER_LENGTH];
  uint8_t _hidfd;
};



#endif // UVCCAMERA_H
