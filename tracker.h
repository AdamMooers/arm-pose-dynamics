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
         * the initial mean centers. Updates the internal cluster. The value for k
         * set in the constructor is used.
         *
         * @param   n           the number of start configuations to run k-means with
         * @param   max_iter    the maximum number of iterations per start configuration
         * @param   epsilon     threshold change in precision between iterations
         */
        void cluster(int n, int max_iter, double epsilon);

        /**
         * Connects the cluster means to form a mesh for analysis. Updates the internal
         * connectivity adjacency matrix.
         *
         * @param   threshold   the threshold, over which two groups are considered connected
         */
        void connect_means(float threshold);

        /**
         * Initializes the tracker. Memory is allocated when at this point to improve performance.
         *
         * @param   k   the number of clusters used for k-means
         */
        tracker(int k);
    
    private:
        int k;

        cv::Mat source_cloud;   // A reference to the original transformed pointcloud
        cv::Mat cluster_ind;    // The clusters for each point in the pointcloud
        cv::Mat centers;        // Centers of the clusters from k-means
        cv::Mat adj_kmeans;     // The adjacency matrix describing the connectivity of the means
};

#endif