# arm-pose-dynamics
A set of joint tracking algorithms for powered, haptic wheelchairs. User pose is calculated from depth camera data.

# Compilation

The following instructions apply for Linux Kernel 4.4.0-xx due to the current limitations of Librealsense.

* Follow the instructions to install librealsense on Ubuntu: [Official Instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
* apt install libsfml-dev
* Navigate to the preferred installation directory
* git clone https://github.com/genuinefaux/arm-pose-dynamics.git
* cd arm-pose-dynamics && make
* See calibration instructions

# Tracking Parameters

See pose.cpp to adjust the following parameters.

POINT_CLOUD_SCALING_CALIB  
POINT_CLOUD_SCALING_TRACKING  
PREFILTER_MANHATTAN_DIST  
PREFILTER_DEPTH_MAX_DIST  
KMEANS_K  
KMEANS_ATTEMPTS  
KMEANS_ITERATIONS  
KMEANS_EPSILON  
KMEANS_CONNECT_THRESHOLD  
LEFT_ARM_START_POS  
RIGHT_ARM_START_POS  
HAND_MAX_DIST_TO_START  
SHOULDER_DXDZ_THRESHOLD  
JOINT_SMOOTHING  
ARM_LOCKED_ANGLE_THESHOLD_D  

# Image Pipeline

# Calibration
Before the tracking can begin, the camera must be calibrated. 


* ./pose or ./pose calibrate

Note: If a segfault occurs in openCV, try rebuilding the entire project with make clean && make
