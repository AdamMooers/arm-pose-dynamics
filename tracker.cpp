/**
 * Author: Adam Mooers
 *
 * Implements the library found in tracker.h.
 */

#include "tracker.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

void tracker::update_point_cloud(pointCloud source)
{
    source_cloud = source.cloud_array;
    cluster_ind.resize(source_cloud.rows);
}

bool tracker::cluster(int n, int max_iter, double epsilon)
{
    // Setup kmeans to terminate after a set number of iterations or when the points have converged
    cv::TermCriteria crit(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, max_iter, epsilon);

    if (source_cloud.rows < k)
    {
        // Not enough data, so clear the buffers
        cluster_ind = cv::Mat(0, 1, CV_32FC1);
        return false;
    }

    // Calculate k-means
    cv::kmeans(source_cloud, k, cluster_ind, crit, n, cv::KMEANS_PP_CENTERS, centers);
    return true;
}

tracker::tracker(int k)
{
    tracker::k = k;
    cluster_ind = cv::Mat(0, 1, CV_32FC1);
    adj_kmeans = cv::Mat(k, k, CV_32FC1);
    centers = cv::Mat(k, 3, CV_32FC1);
}