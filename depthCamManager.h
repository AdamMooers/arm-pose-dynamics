/**
 * Author: Adam Mooers
 *
 * Manages the depth cameras in a library-independent manner. This allows
 * the codebase to take advantages of new depth sensors from different vendors
 * without requiring a complete rewrite. The library is designed so that it can
 * be used with multiple cameras at the same time.
 */

#ifndef DEPTHCAMMANAGER_H
#define DEPTHCAMMANAGER_H

#include <librealsense/rs.hpp>

/**
 * Manages a depth camera over its lifetime. Also provides support for conversion
 * to point clouds, multi-camera management, etc.
 */
class depth_cam
{
    public:
        /**
         * Intializes the camera to the default device. Currently, multiple
         * devices are not supported, but support can be added easily by modifying
         * this function.
         */
        depth_cam * depth_cam_init();

        /**
         * Converts the given depth frame into a point cloud.
         */

        /**
         * Removes the background from an image given from a known start point.
         * The start point is in the transformed point cloud domain, so a known
         * point cloud must be established, along 
         */

    private:
        rs_device * dev;              // Pointer to info about the device itself
        rs_intrinsics depth_intrin;   // Depth intrinics of the frame, updates with each new frame
}



class point_cloud
{
    public:
        // Set Instance at index (possible realloc)
        // Transform Point
        // Tranform cloud
        // Set transform
        // Set transform from file
        // Clip
        // Normalize
        // Get calibration transform from plane

    private:
        // length size
        // cvVector array
}

 #endif