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
#include "types.hpp"

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
         * Converts the given depth frame into a point cloud from the camera frame of reference.
         */

        /**
         * Removes the background from the source image given based on a position-based heuristic
         * in the source image pixel domain. The resulting image is stored in the modifier
         * buffer the is used when point cloud conversion occurs.
         * 
         * @param   startLoc    the normalized starting position of the algorithm. (0,0) is in the top-left
         *                      of the image. (1,1) is the bottom-right most pixel of the image.
         *
         * @param   slope       the pixelwise direction to travel while searching for the start position.
         *                      This vector is added to the current position each loop iteration.
         */
         bool removeBackground(float startLoc, Point slope)

    private:
        rs_device * dev;            // Pointer to info about the device itself
        rs_intrinsics depth_intrin; // Depth intrinics of the frame, updates with each new frame

        const uint16_t * srcImg;            // A reference to the source image  
        uint16_t * modifiedSrc = nullptr;   // A reference to the modified image
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