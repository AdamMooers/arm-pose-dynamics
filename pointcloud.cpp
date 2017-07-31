/**
 * Author: Adam Mooers
 *
 * Implements the library found in pointcloud.h.
 */

#include "pointcloud.h"

void pointcloud::clear(void)
{
    cur_size = 0;   // Nothing in the array now
}

void pointcloud::add_point(Vec3i point)
{
    cloud_array[cur_size] = point;
    cur_size++;
}

point_cloud::point_cloud(int max_size)
{
    cloud_array = new Vec3i[max_size];
}

point_cloud::~point_cloud(int max_size)
{
    if (cloud_array != nullptr)
    {
        delete cloud_array;
    }
}