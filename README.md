# CubeRover

This repository contains software relating to the prototype curerover built around a ODROID for accelerating development of structure-from-motion (SfM) based reconstruction of blast ejecta on the lunar surface.

## Requirements

Ubuntu 14.04.5 LTS (Trusty Tahr)

ROS Indigo

OpenCV 2.4.9 (wtih custom patch)

e-con Systems custom OpenCV 2.4.9 patch (for custom Y16 grayscale image format)

## Personal Computer Installation

### ROS Indigo

Follow the standard installation steps found at http://wiki.ros.org/indigo/Installation/Ubuntu. Depending on the target application, download the Desktop-Full for a personal computer and the Desktop Install for a more embedded applicaton.

### Dependencies
```
sudo apt-get install ros-indigo-libuvc-camrea
sudo apt-get install v4l-utils
```

### OpenCV 2.4.9 with e-con Systems Patch

The steps outlined need to be followed closely to ensure that the custom image format that is provided by e-con Systems is properly built with OpenCV so that the ROS node can read and publish the grayscale images streaming from the camera.

#### 1. Clone OpenCV source into the dependencies folder
```
cd dependecies
git clone https://github.com/opencv/opencv.git
git checkout tags/2.4.9
```
#### 2. Unzip the e-con Systems patch source and install the source code patch
```
unzip e-con_Modified_OpenCV.zip
cd e-con_Modified_OpenCV
sudo chmod +x install_econ_patch.sh
sudo ./install_econ_patch.sh
```

#### 3. Create a patched opencv installation folder in the dependencies folder
```
cd ../ && mkdir opencv_econ
```

#### 4. Create enviornment variable that points to the installation folder
```
gedit ~/.bashrc
```
Add the following line to the bottom of your bashrc
```
export OPENCV_ECON_PATH=/path/to/opencv_econ
```
where you replace '/path/to/opencv_econ' with the actual fixed path to the folder. This is key in allowing cmake to find the patched OpenCV files.

#### 5. Open a new terminal and enter the opencv source folder and create and enter a build folder
```
mkdir -p opencv/build
cd opencv/build
````

#### 6. Run cmake for opencv
```
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_V4L=ON -D WITH_LIBV4L=OFF -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_INSTALL_PREFIX=../../opencv_econ ..
```
Notice: The CMAKE_INSTALL_PREFIX is set to the opencv_econ folder created. This is where the patched opencv will live and be referenced as to no interfere with the opencv installed by ROS.

#### 7. Make and install opencv
```
sudo make -j4 install
```

### Udev Rules

#### 1. Enter the udev_rules directory in the dependencies folder
```
cd dependenceis/udev_rules
```

#### 2. Run the udev rules installation script
```
sudo chmod +x install-udev-rules.sh
sudo ./install-udev-rules.sh
```

Unplug and plug the camera back into your computer for the udev permissions to take effect.

## Raw Converter Usage

The raw converter is used to convert the .raw 12-bit grayscale resolution images into jpgs.

#### 1. Build the entire catkin_ws
```
cd catkin_ws
catkin_make
````

#### 2. Navigate to the built executable directory
```
cd devel/lib/raw_converter
```

#### 3. Execute the raw converter, passing in the path to the directoy of raw images
```
./raw_converter -d /path/to/images
```
where you replace '/path/to/images' with the real path to the directory containing the .raw images you wish to convert. The converted images will show up in a directory named 'jpgs' in the raw images path directory.

## Setting up the ODROID

Kernel building and installation instructions sourced from:
https://github.com/umiddelb/armhf/wiki/How-To-compile-a-custom-Linux-kernel-for-your-ARM-device

#### Download 14.04 image
```
com.odroid.com/sigong/nf_file_board/nfile_board_view.php?keyword&tag&bid=241
```

#### Flash the OS image
```
sudo dd if=<my/odroid/image.img> of=</dev/path/of/card> bs=1M conv=fsync
sync
````
Replace ```my/odroid/image.img``` with the path to the image downloaded in the last step. ```/dev/path/of/card``` is the path to the target storage device in terms of the /dev folder. Use ```sudo lsblk``` or ```sudo fdisk -l``` to figure out what the right ```/dev/*``` path is.

#### Install the flash memory in the ODROID and boot it up

#### Install dependencies
```
sudo apt-get -y install bc curl gcc git libncurses5-dev lzop make u-boot-tools
```

### Install and set gcc-5 as your default compiler
```
sudo apt-get -y install python-software-properties;
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test;
sudo apt-get update
sudo apt-get -y install gcc-5 g++-5
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 50
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 50
```

Verify it worked by calling the following, it should say something about gcc-5.x
```
gcc --version
```

#### Download linux kernel v4.2 for the ODROID XU4 from
```
https://github.com/tobetter/linux/archive/odroidxu4-v4.2.tar.gz
```

#### Uncompress the kernel source and build the configuration file for the xu4
```
cd linux-odroidxu4-4.2.y
make odroidxu4_defconfig
```

#### Configure the kernel using the menuconfig for HIDRAW enabled
```
make menuconfig
```
Navigate to and enable the following
```
Device Drivers ---> HID support ---> /dev/hidraw raw HID device support
Device Drivers ---> Multimedia support ---> Cameras/video grabbers support
Device Drivers ---> Multimedia support ---> Enable advanced debug functionality on V4L2 drivers
Device Drivers ---> Multimedia support ---> Media USB Adapters
Device Drivers ---> Multimedia support ---> V4L platform devices
Device Drivers ---> Multimedia support ---> Media USB Adapters ---> USB Video Class (UVC)
Device Drivers ---> Multimedia support ---> Media USB Adapters ---> UVC input events device support
```

#### Make the kernel and its modules/firmware and install
```
make -j 8 zImage dtbs modules
sudo cp arch/arm/boot/zImage arch/arm/boot/dts/*.dtb /media/boot
sudo make modules_install
sudo make firmware_install
sudo make headers_install INSTALL_HDR_PATH=/usr
kver=`make kernelrelease`
sudo cp .config /boot/config-${kver}
cd /boot
sudo update-initramfs -c -k ${kver}
sudo mkimage -A arm -O linux -T ramdisk -a 0x0 -e 0x0 -n initrd.img-${kver} -d initrd.img-${kver} uInitrd-${kver}
sudo cp uInitrd-${kver} /media/boot/uInitrd
```

## Initializing the CAN bus interface

#### Define CAN socket
Ensure that the USB-to-CAN adapter is plugged in and powered. The following will set up the adapter as a network socket and define its bit rate to 1Mbps.
```
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```
Run ```ifconfig``` to ensure the device has been successfully created and brought up.
