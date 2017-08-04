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
         * @return  whether or not kmeans was able to run (was the point cloud size > k?)
         */
        bool cluster(int n, int max_iter, double epsilon);

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

        cv::Mat cluster_ind;    // The clusters for each point in the pointcloud
        cv::Mat centers;        // Centers of the clusters from k-means
        cv::Mat adj_kmeans;     // The adjacency matrix describing the connectivity of the means

    private:
        int k;                  // Number of clusters in the simulation
        cv::Mat source_cloud;   // A reference to the original transformed pointcloud
};

class arm
{
    public:
        // Contains the kmeans indices in the arm mesh from hand (at the front)
        // to shoulder (at the back)        
        std::list<int> kmean_ind;
        int elbow_kmean_ind;

        /**
         * Calculates the shoulder location from the given hand location. The kmeans
         * center closest to the start position (but with a greater z) is taken as the 
         * start node. The algorithm then progresses up the graph until the change in
         * x is greater than a threshold or if no more "up" options remain. The next
         * selected point is the furthest neighbor from the global mean that is in the
         * positive direction.
         *
         * @param   source              the tracker to use for data
         * @param   start_pos           the approximate location of the hand (1x3)
         * @param   max_dist_to_start   the maximum distance to the start position the start node can be
         * @param   dx_threshold        the threshold in dx to terminate the algorithm
         * @return  whether or not the arm was identified correctly (was dx_threshold the terminating condition?)
         */
        bool update_arm_list(cv::Mat start_pos, float max_dist_to_start, float dx_threshold);

        /**
         * Identifies the elbow by finding the point in kmean_ind which has
         * minimum in difference between the shoulder and the hand.
         */
        bool update_elbow(void);
};

#endif