/**
 * Author: Adam Mooers
 *
 * Implements the library found in pointcloud.h.
 */

#include <iostream>
#include "pointCloud.h"

void pointCloud::clear(void)
{
    cloud_array.resize(0);   // Nothing in the array now
}

void pointCloud::add_point(cv::Mat point)
{
    // printf("cursize: %d\n", cur_size);
    cloud_array.push_back(point);
}

void pointCloud::get_transform_from_cloud(void)
{
    if (cloud_array.rows < cloud_array.cols)
    {
        return;
    }

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
    
    std::cout << beta << "\n";

    //exit(0);
}

pointCloud::pointCloud(void)
{
    cloud_array = cv::Mat(0, 3, CV_32FC1);
}