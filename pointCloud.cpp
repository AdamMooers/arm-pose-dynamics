/**
 * Author: Adam Mooers
 *
 * Implements the library found in pointcloud.h.
 */

#include <iostream>
#include "pointCloud.h"

void pointCloud::clear(void)
{
    cur_size = 0;   // Nothing in the array now
}

void pointCloud::add_point(cv::Vec3f point)
{
   // printf("cursize: %d\n", cur_size);
    cloud_array[cur_size] = point;
    cur_size++;
}

pointCloud::pointCloud(int max_size)
{
    cloud_array = new cv::Vec3f[max_size];
    clear();
}

pointCloud::~pointCloud()
{
    if (cloud_array != nullptr)
    {
        delete[] cloud_array;
    }
}