/**
 * Author: Adam Mooers
 *
 * Implements the library found in pointcloud.h.
 */

#include "pointCloud.h"

void pointCloud::clear(void)
{
    cur_size = 0;   // Nothing in the array now
}

void pointCloud::add_point(cv::Vec3i point)
{
    cloud_array[cur_size] = point;
    cur_size++;
}

pointCloud::pointCloud(int max_size)
{
    cloud_array = new cv::Vec3i[max_size];
}

pointCloud::~pointCloud()
{
    if (cloud_array != nullptr)
    {
        delete cloud_array;
    }
}