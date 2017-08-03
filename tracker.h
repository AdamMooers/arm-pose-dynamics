/**
 * Author: Adam Mooers
 *
 * Computes the joint tracking given a normalized point cloud representing the
 * user. The general idea behind the algorithm is that having the hands eliminates
 * several degrees-of-freedom from the model. These serve as the start point to
 * building the skeleton model of the user. kmeans is used to cluster groups
 * of input points.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include "opencv2/core/core.hpp"
#include "pointCloud.h"


class tracker
{
    public:
        /**
         * Set the point cloud source for the tracker. Call this function
         * each time the source cloud is modified, before tracking.
         *
         * @param   source    the point cloud to refer to.
         */
        void update_point_cloud(pointCloud source);

        /**
         * Uses K-means clustering with the given number of iterations and clusters
         * to determine how the point cloud is connected. kmeans++ is used to set
         * the initial mean centers. Updates the internal cluster.
         *
         * @param   k   the number of clusters in the cloud
         * @param   n   the number of iterations to run k-means on the point-cloud
         */
        void cluster(int k, int n);

        /**
         * Connects the cluster means to form a mesh for analysis. Updates the internal
         * connectivity adjacency matrix.
         *
         * @param   threshold   the threshold, over which two groups are considered connected
         */
        void connect_means(float threshold);
    
    private:
        cv::Mat source_cloud;
        cv::Mat cluster_ind;
};

#endif