/**
 * Author: Adam Mooers
 *
 * Implements the library found in pointcloud.h.
 */

#include <iostream>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "pointCloud.h"

void pointCloud::clear(void)
{
    cloud_array.resize(0);   // Nothing in the array now
}

void pointCloud::add_point(cv::Mat point)
{
    cloud_array.push_back(point);
}

void pointCloud::get_transform_from_cloud(void)
{
    // Is there enough data to work with?
    if (cloud_array.rows < cloud_array.cols)
    {
        return;
    }

    cv::Mat z_vec = get_normal_from_cloud();
    cv::Mat y_vec_props, y_vec; 
    cv::Mat x_vec;
    cv::Mat calib_origin;

    cv::fitLine(cloud_array, y_vec_props, CV_DIST_L2, 0, line_fitting_reps, line_fitting_aeps);

    // Extract the y-vector
    y_vec = y_vec_props(cv::Rect(0,0,1,3)).t();

    // Calculate the x-axis
    x_vec = z_vec.cross(y_vec);

    // Rotate the pointcloud about this point
    float z_rot_theta = atan2(x_vec.at<float>(0,1), x_vec.at<float>(0,0));
    float y_rot_theta = -atan2(x_vec.at<float>(0,2), cv::norm(x_vec(cv::Rect(0,0,2,1))));

    // Calculate the first two rotational transforms
    float z_rot_arr[3][3] = { {   (float)cos(z_rot_theta),   (float)-sin(z_rot_theta),  0},
                              {   (float)sin(z_rot_theta),   (float)cos(z_rot_theta),   0},
                              {   0,                         0,                         1}};

    float y_rot_arr[3][3] = { {   (float)cos(y_rot_theta),   0,   (float)sin(y_rot_theta)},
                              {   0,                         1,                         0},
                              {   (float)-sin(y_rot_theta),  0,   (float)cos(y_rot_theta)}}; 

    cv::Mat z_rot(3, 3, CV_32FC1, &z_rot_arr);
    cv::Mat y_rot(3, 3, CV_32FC1, &y_rot_arr);

    // Combine the first two rot. transforms
    calib_transform = z_rot*y_rot;

    // Rotate axis to determine final rotation
    cv::Mat z_transformed_yz = z_vec*calib_transform;
    float x_rot_theta = -atan2(z_transformed_yz.at<float>(0,1), z_transformed_yz.at<float>(0,2));

    float x_rot_arr[3][3] = { {   1,                         0,                         0},
                              {   0,   (float)cos(x_rot_theta),  (float)-sin(x_rot_theta)},
                              {   0,   (float)sin(x_rot_theta),  (float)cos(x_rot_theta) }};

    cv::Mat x_rot(3, 3, CV_32FC1, &x_rot_arr);

    // Combine the x-transform
    calib_transform = calib_transform*x_rot;  

    // Transform the pointcloud
    cloud_array=cloud_array*calib_transform;

    // Find the mean: This works best if the point cloud density is normalized
    cv::reduce(cloud_array, calib_origin, 0, CV_REDUCE_AVG);
}

cv::Mat pointCloud::get_normal_from_cloud(void)
{
    // The X matrix needs a column of ones
    cv::Mat X_components[] = {
        cv::Mat::ones(cloud_array.rows, 1, cloud_array.type()),         // just ones
        cloud_array(cv::Rect(0,0,cloud_array.cols-1,cloud_array.rows)), // x and y columns
    };

    cv::Mat X, X_trans, beta, y;

    y = cloud_array(cv::Rect(cloud_array.cols-1,0,1,cloud_array.rows));

    cv::hconcat(X_components, 2, X);

    // Get x'
    cv::transpose(X, X_trans);

    // Calculate the least-squares plane
    beta = (X_trans*X).inv()*(X_trans*y);

    // Extract the normal
    float normal_arr_z[] = {beta.at<float>(0,1), beta.at<float>(0,2), -1.0f};

    // Convert the normal to cv::mat format
    cv::Mat normal_z_src(1, 3, CV_32FC1, &normal_arr_z);
    cv::Mat normal_z; 

    normal_z_src.copyTo(normal_z);
    
    return normal_z;
}

pointCloud::pointCloud(void)
{
    cloud_array = cv::Mat(0, 3, CV_32FC1);
    calib_rot_transform = cv::Mat(3,3, CV_32FC1);
    calib_transform = cv::Mat(3,3, CV_32FC1);
}