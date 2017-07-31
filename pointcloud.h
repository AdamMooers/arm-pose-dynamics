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

class point_cloud
{
    public:
        // Transform Point
        // Tranform cloud
        // Set transform
        // Set transform from file
        // Clip
        // Normalize
        // Get calibration transform from plane

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
        void add_point(Vec3i point);

        /**
         * Creates a new point cloud of the given buffer size. The object
         * which is stored in the point cloud can be up to this size.
         *
         * @param   max_size    the maximum buffer size
         */
        point_cloud (int max_size);

    private:
        int max_size;                       // The maximum size of the array
        int cur_size;                       // The currently-filled portion of the array
        cv::Vec3i *cloud_array = nullptr;   // The array for the points in the cloud
}

#endif