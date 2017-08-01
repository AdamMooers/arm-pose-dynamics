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

pointCloud::pointCloud(void)
{
    cloud_array = cv::Mat(0, 3, CV_32FC1);
}