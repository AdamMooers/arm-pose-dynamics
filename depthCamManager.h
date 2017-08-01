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
#include "pointCloud.h"

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
         * Converts the given depth frame into a point cloud from the camera frame of reference.
         */
        void to_depth_frame(void);

        /**
         * Removes the background from the captured frame by segmenting the image into groups of close
         * pixels (based on distance). The largest group is kept. All other groups are erased. The result
         * overwrites the pipeline_src buffer.
         *
         * @param   maxDist     the maximum distance between which two points can be in the same group (meters, depth)
         * @param   manhattan   the neighborhood to explore is within this manhattan distance of the point
         */
        void filter_background(float maxDist, int manhattan);

        cv::Mat cur_src;                    // The image in the current state of the pipeline
        pointCloud * cloud = nullptr;       // The point cloud for the current frame

        /**
         * @param scale_factor Sets the scale factor of the depth camera.
         */
        depth_cam(float scale_factor);

        ~depth_cam( void );

    private:
        float scale_factor;                 // The scale factor to apply to the depth image before processing
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

        /**
         * Zeroes all elements of the image matrix except for those with the given value. Those
         * within the given value are replaced by the selected alternative.
         *
         * @param   input_img   the image to overwrite
         * @param   target      the value to target
         * @param   output      the image to filter
         */
        int mask_by_cluster_id(cv::Mat& cluster_img, int32_t cluster_id, cv::Mat& output_img);

        const uint16_t * srcImg;            // A reference to the source image  
};

 #endif