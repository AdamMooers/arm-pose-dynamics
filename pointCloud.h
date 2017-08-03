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

        /**
         * Saves the calibration transform to the given file in XML format.
         * The matrix is saved in floating-point format. Both rotation and
         * translation are saved by name in the persistence file.
         * The load_calibration_matrix file will load any files saved by
         * this function. Make sure the file has a format of *.xml for
         * *.yml. These are the two format recognized by openCV.
         *
         * @param   file_path   the path to the file to create/overwrite
         */
        void save_calibration_matrix(const char* filename);

        /**
         * Loads the calibration matrix described by save_calibration_matrix
         * into the calib_rot_transform and calib_origin.
         *
         * @param   file_path   the path to the file to read
         */
        void load_calibration_matrix(const char* filename);

        /**
         * Transforms the entire cloud using the current rotation and translation
         * matrices. point_cloud = point_cloud*R + T. Be sure to load the desired
         * transform from file (load_calibration_matrix(...)) or from a calibration
         * cube first.
         */
        void transform_cloud(void);

        /** 
         * Prompts the user for the manual offset to add to the calibration
         * translation. The result entered by the user is added immediately
         * after the prompt finishes. The result can then be preserved with the
         * save_calibration_matrix function.
         */
        void prompt_for_manual_offset(void);

        cv::Mat cloud_array;            // The current point cloud

        /**
         * Initializes the point cloud. The homogeneous transform matrix equivalent 
         * [calib_rot_transform, calib_origin; 0, 1] is loaded with the identity matrix.
         */
        pointCloud(void);

    private:
        int cur_size;                   // The currently-filled portion of the array
        cv::Mat calib_rot_transform;    // The rotational transform from the point-cloud
        cv::Mat calib_origin;           // The translation from the camera to the box center

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

        /**
         * Applies the selected translation to the selected matrix.
         * The given translation (1xn) is added to each row of the matrix (mxn).
         * Matlab-equivalent operation: M = M + repmat(T, m, 1);
         *
         * @param   translation     the translation to apply (T)
         * @param   matrix          the matrix to apply the transformation to (M)
         */
        static void apply_translation(cv::Mat translation, cv::Mat& matrix);
};

#endif