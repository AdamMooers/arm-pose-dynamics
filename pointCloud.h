/**
 * Author: Adam Mooers
 *
 * Manages a point-cloud for real-time use. The library is designed
 * so that a single point-cloud object can be reused between frames
 * since the pointcloud can be quite large. point_cloud handles standard
 * pointcloud operations such as clipping and normalizing the pointcloud.
 */

#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "opencv2/core/core.hpp"

class pointCloud
{
    public:
        // Transform Point
        // Tranform cloud
        // Set transform
        // Set transform from file
        // Clip
        // Normalize

        /**
         * Gets the calibration transform from the current point-cloud.
         * Linear least-squares planar regression is used to identify the 
         * X-Y plane. 
         */
        void get_transform_from_cloud(void);

        /**
         * Logically clears the pointcloud. Use this in-between frames.
         */
        void clear(void);

        /**
         * Adds a new point to the end of the pointcloud array.
         * The point is added as given. No transformation occurs.
         *
         * @param   point       the point to add
         */
        void add_point(cv::Mat point);

        pointCloud(void);

    private:
        int cur_size;                   // The currently-filled portion of the array
        cv::Mat cloud_array;            // The current point cloud
        cv::Mat calib_rot_transform;    // The rotational transform from the point-cloud
        cv::Mat calib_transform;        // The homogeneous transform to correct the pointcloud data   

        const double line_fitting_reps = 0.01;  // Radius accuracy parameter for line fitting
        const double line_fitting_aeps = 0.01;  // Angle accuracy parameter for line fitting

        /**
         * Finds the transform plane using least-squares regression. The
         * normal is the new z-axis of the system. Uses cloud_array as the
         * source.
         *
         * @return  parameters of z = Ax + By + C as [C A B]
         */
        cv::Mat get_normal_from_cloud(void);
};

#endif