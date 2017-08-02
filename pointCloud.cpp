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

    float z_rot_theta = 

    // Find the mean: This works best if the point cloud density is normalized
    cv::reduce(cloud_array, calib_origin, 0, CV_REDUCE_AVG);

    std::cout << "cloud_array= " << cloud_array << "\n";
    std::cout << "y_vec= " << y_vec << "\n";
    std::cout << "z_vec= " << z_vec << "\n";
    exit(0);
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
    float normal_arr[] = {beta.at<float>(0,1), beta.at<float>(0,2), -1.0f};

    // Convert the normal to cv::mat format
    cv::Mat normal(1, 3, CV_32FC1, &normal_arr);
    
    return normal_arr;
}

pointCloud::pointCloud(void)
{
    cloud_array = cv::Mat(0, 3, CV_32FC1);
}