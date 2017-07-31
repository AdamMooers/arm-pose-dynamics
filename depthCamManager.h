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
#include "opencv2/core/core.hpp"

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
        bool depth_cam_init( void );

        /**
         * Activates the depth camera stream. 
         */
        void start_stream( void );

        /**
         * Polls the device for the next frame. This function polls in the same thread 
         * as it is called in. If no error occurs, the manager will have an internal
         * reference to the latest depth frame from the camera. Note that the stream
         * must be started before this function is called.
         */
        void capture_next_frame( void );

        /**
         * Retrieves a reference to the next frame. 

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
         *
         * @param   maxDist     the maximum distance allowed for two points to be considered neighbors (in meters)
         */
        void remove_background(float startLoc, cv::Point slope);

        /**
         * Converts the given depth frame into a point cloud from the camera frame of reference.
         */
        void to_depth_frame( void );

        /**
         * Removes the background from the captured frame by segmenting the image into groups of close
         * pixels (based on distance). The largest group is kept. All other groups are erased. The result
         * overwrites the pipeline_src buffer.
         *
         * @param   maxDist     the maximum distance between which two points can be in the same group (meters, depth)
         * @param   manhattan   the neighborhood to explore is within this manhattan distance of the point
         */
        void filter_background( float maxDist, int manhattan );

        cv::Mat cur_src;        // The image in the current state of the pipeline

        depth_cam( void );
        ~depth_cam( void );

    private:
        rs::device * dev = nullptr;         // Currently the library only supports a single depth cam
        rs::context * ctx = nullptr;        // Manages all of the realsense devices
        rs::intrinsics depth_intrin;        // Depth intrinics of the frame, updates with each new frame

        /**
         * Runs BFS on the given image starting from a given pixel and expanding outwards.
         * All pixels in the same group are marked with the index in the output image and
         * zeroed in the input image. Maxdist and manhattan are treated the same way as
         * in filter_background.
         *
         * @param   x           the x-coordinate of the start pixel (zero-based)
         * @param   y           the y-coordinate of the start pixel (zero-based)
         * @param   cluster_id  the id of the cluster
         * @param   input_img   the source image to partition
         * @param   cluster_img the output image containing clusters
         * @param   maxDist     the maximum distance between which two points can be in the same group (meters, depth)
         * @param   manhattan   the neighborhood to explore is within this manhattan distance of the point
         *
         * @return  the area of the cluster including the initial pixel in number of pixels
         */
        int img_BFS(int x, int y, int cluster_id, cv::Mat& input_img, cv::Mat& cluster_img, float maxDist, int manhattan);   

        const uint16_t * srcImg;            // A reference to the source image  
};


/*
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
*/

 #endif