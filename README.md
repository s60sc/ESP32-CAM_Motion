# ESP32-CAM_Motion

ESP32 Camera extension to allow the camera to detect movement using image centre of mass shift, whilst still retrieving large JPEG images.

## Purpose

This allows the camera to detect movement eg for security camera purposes, without additional hardware such as a PIR. It also allows the light level to be determined by the camera eg to switch on the lamp, without additional hardware such as an LDR.

Movement detection using image centre of mass shift is more effective than simply comparing bitmap changes as the latter is subject to spurious movement such as camera noise, particularly in low light, transient movements such as leaves rustling or rain, and illumination changes such as passing clouds.

This extension can be used in conjunction with the [ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD) repository for a more complete solution.

## Design

JPEG images of any size are retrieved from the camera and 1 in N images are sampled on the fly for movement by decoding them to very small bitmap images from which the image centre of mass is calculated and compared to the previous sample.
The table shows typical time taken to decode and analyse a frame retrieved from the OV2640 camera.

Frame Size | Time in ms
------------ | ------------- 
QQVGA | 20
HQVGA | 15
QVGA | 30
CIF | 40
VGA | 70
SVGA | 100
XGA | 200
SXGA | 300
UXGA | 400

QQVGA is slower because it uses a different scale factor.

In the [ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD) sketch, for movement detection a high sample rate of 1 in 2 is used. When movement has been detected, the rate for checking for movement stop is reduced to 1 in 10 so that the JPEGs can be captured with only a small overhead.

## Installation and Use

Download the files into the Arduino IDE sketch location, removing `-master` from the folder name.  The included Arduino sketch provides the basic functionality for testing and calibrating the extension using the OV2640 camera.

The following monitoring parameters can be modified:
* `#define MOTION_SEQUENCE 5` The number of sequential frame changes that constitute a valid movement
* `#define NIGHT_SEQUENCE 100` The number of sequential dark frames that constitute night time

The web page in [ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD) includes additional options:
* __Motion Sensitivity__ sets how much centre of mass shift is needed to constitute a movement. The faster that frames are received the higher the value should be for more sensitivity
* __Night Switch__ sets the ambient light level that constitutes night time. __Ambient Light__ displays the current light level

The file `mjpeg2sd.cpp` in [ESP32-CAM_MJPEG2SD](https://github.com/s60sc/ESP32-CAM_MJPEG2SD) has the parameter:
* `#define POST_MOTION_TIME 2` which sets the number of seconds the recording continues after motion stop.

