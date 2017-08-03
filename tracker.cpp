/**
 * Author: Adam Mooers
 *
 * Implements the library found in tracker.h.
 */

#include "tracker.h"

void tracker::update_point_cloud(pointCloud source)
{
    source_cloud = source.cloud_array;
}