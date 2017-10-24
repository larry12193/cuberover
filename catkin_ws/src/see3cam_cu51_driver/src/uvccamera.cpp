/*******************************************************************************
 * uvccamera.cpp - UVC camera interface for see3cam_cu51 camera from econ
 *                 systems
 *
 * Carnegie Mellon University
 * Author: Lawrence Papincak
 *
 * Adapted from econ Systems QtCam viewer application
 * https://github.com/econsysqtcam/qtcam
 ******************************************************************************/

#include "uvccamera.h"

// Constructor
uvccamera::uvccamera(std::string port) {
  _port = port;
}

// Destructor
uvccamera::~uvccamera() {
  if(_hidfd >= 0) {
    close(_hidfd);
  }
}

unsigned int uvccamera::getTickCount() {

  struct timeval tv;
  if(gettimeofday(&tv, NULL) != 0)
      return 0;

  return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

// Initialize the camera UVC extension unit
bool uvccamera::initCamera() {
  // Open hidraw device
  _hidfd = open(_port.c_str(), O_RDWR | O_NONBLOCK);

  if(_hidfd < 0) {
    perror("Unable to open hidraw device!");
    return false;
  }
  return true;
}

// Enable master mode, allowing for changing exposure
bool uvccamera::enableMasterMode() {

  int ret = 0;

  // Make sure the port was opened properly
  if(_hidfd < 0) {
    return false;
  }

  // Initialize output buffer
  memset(_out_buf, 0x00, sizeof(_out_buf));

  // Send the report number
  _out_buf[1] = ENABLEMASTERMODE;
  ret = write(_hidfd, _out_buf, BUFFER_LENGTH);
  if( ret < 0 ) {
    perror("write - enable master mode");
    return false;
  }
  return true;
}

// Get current camera exposure
bool uvccamera::getExposure() {

  if( _hidfd < 0 ) {
    perror("No extension unit initialized - get exposure");
    return false;
  }

  int ret = 0;
  bool timeout = false;
  unsigned int start, end = 0;

  // Initialize output buffer
  memset(_out_buf, 0x00, sizeof(_out_buf));

  _out_buf[1] = CAMERA_CONTROL_51;
  _out_buf[2] = GET_EXPOSURE_VALUE;
  ret = write(_hidfd, _out_buf, BUFFER_LENGTH);
  if( ret < 0 ) {
    perror("write - get exposure");
    return false;
  }

  // Get starting time
  start = getTickCount();

  // Wait for data to come in or timeout to occur
  while( !timeout ) {
    // Read input from camera
    ret = read(_hidfd, _in_buf, BUFFER_LENGTH);
    // If bytes were returned, process them
    if( ret > 0 ) {
      // Check if the message makes sense
      if( _in_buf[0] == CAMERA_CONTROL_51 &&
          _in_buf[1] == GET_EXPOSURE_VALUE) {
        // Check if the operation was successful
        if(_in_buf[4] == EXP_SUCCESS) {
          // Build exposure value from returned bytes
          curExposure = (_in_buf[3]<<8) + _in_buf[2];
          return true;
        } else if( _in_buf[4] == EXP_FAIL ) {
          perror("getExposure - failed to read from device");
          return false;
        }
      }
    }
    // Get current time and compare to start time to monitor timeout
    end = getTickCount();
    if( end - start > TIMEOUT ) {
      timeout = true;
      perror("read timeout - get exposure");
      return false;
    }
  }
  return false;
}

// Set camera exposure
bool uvccamera::setExposure(uint16_t newValue) {

  if( _hidfd < 0 ) {
    perror("No extension unit initialized - get exposure");
    return false;
  }

  // Check if desired exposure setting is within range
  if( newValue >= 1 && newValue <= 30000 ) {
    int ret = 0;
    bool timeout = false;
    unsigned int start, end = 0;

    // Initialize output buffer
    memset(_out_buf, 0x00, sizeof(_out_buf));

    _out_buf[1] = CAMERA_CONTROL_51;
    _out_buf[2] = SET_EXPOSURE_VALUE;
    _out_buf[3] = newValue & 0xFF;    // LSB of new exposure value
    _out_buf[4] = newValue >> 8;      // MSB of new exposure value

    ret = write(_hidfd, _out_buf, BUFFER_LENGTH);
    if( ret < 0 ) {
      perror("write - set exposure");
      return false;
    }

    // Get starting time
    start = getTickCount();

    // Wait for data to come in or timeout to occur
    while( !timeout ) {
      // Read input from camera
      ret = read(_hidfd, _in_buf, BUFFER_LENGTH);
      // If bytes were returned, process them
      if( ret > 0 ) {
        // Check if the message makes sense
        if( _in_buf[0] == CAMERA_CONTROL_51  &&
            _in_buf[1] == SET_EXPOSURE_VALUE &&
            _in_buf[2] == _out_buf[3]        &&
            _in_buf[3] == _out_buf[4]) {
          // Check if the operation was successful
          if(_in_buf[4] == EXP_SUCCESS) {
            return true;
          } else if( _in_buf[4] == EXP_FAIL ) {
            perror("getExposure - failed to read from device");
            return false;
          }
        }
      }
      // Get current time and compare to start time to monitor timeout
      end = getTickCount();
      if( end - start > TIMEOUT ) {
        timeout = true;
        perror("read timeout - get exposure");
        return false;
      }
    }
  } else {
    perror("setExposure - exposure out of range");
  }
  return false;

}
// bool setBrightness();
// bool getBrightness();
