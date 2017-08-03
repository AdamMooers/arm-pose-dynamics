/**
 * Author: Adam Mooers
 *
 * Implements the library found in tracker.h.
 */

#include "tracker.h"
#include <opencv2/imgproc/imgproc.hpp>

void tracker::update_point_cloud(pointCloud source)
{
    source_cloud = source.cloud_array;
    cluster_ind.resize(source_cloud.rows);
}

void tracker::cluster(int n, int max_iter, double epsilon)
{
    // Setup kmeans to terminate after a set number of iterations or when the points have converged
    cv::TermCriteria crit(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, max_iter, epsilon);

    // Calculate k-means
    cv::kmeans(source_cloud, k, cluster_ind, crit, n, cv::KMEANS_PP_CENTERS, centers);
}

tracker::tracker(int k)
{
    cluster_ind = cv::Mat(0, 1, CV_32FC1);
    adj_kmeans = cv::Mat(k, k, CV_32FC1);
    centers = cv::Mat(k, 3, CV_32FC1);
}