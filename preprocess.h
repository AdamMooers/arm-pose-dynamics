/**
 * Author: Adam Mooers
 *
 * Preprocesses point cloud data from a generic depth frame. This includes removing 
 * background noise, converting to a point cloud, transforming the  point cloud, 
 * applying bounds to the data, and normalizing the data. The library supports
 * multiple instances for multiple cameras (and is therefore threadsafe). It is intended
 * for real-time use (e.g. low latency), but may be used for post-processing as well. 
 */
 #ifndef PREPROCESS_H
 #define PREPROCESS_H

 #include "depthCamManager.h"

 typedef struct frameController
 {

     rs_intrinsics depth_intrin; // Intrinsics of the depth frame
 }

 #endif