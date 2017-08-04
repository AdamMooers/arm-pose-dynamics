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

    int flags = cv::KMEANS_PP_CENTERS;  // Use kmeans++ heuristic
    
    if (cluster_ind.rows == 0)
    {
        printf("Triggered\n");
        flags += cv::KMEANS_USE_INITIAL_LABELS;
    }

    // Calculate k-means
    cv::kmeans(source_cloud, k, cluster_ind, crit, n, flags, centers);
    return true;
}

void connect_means(float threshold)
{
	cv::Mat k_histogram = cv::Mat::zeros(k, 1, CV_32FC1);
	
	// Zero out adj_kmeans
	
	// for each point in the cloud
	for (int r_cl=0; r_cl<source_cloud.rows; r_cl++)
	{
		// int curKInd = cluster_ind(r_cl,0);
		// dist2home = sum(dot(centers(curKInd,0), source_cloud(r_cl)))
		
		// Update histogram for normalization
		// k_histogram(curKInd,0) += 1
		
		// for each k-mean
		for(int r_center=0; r_center<centers.rows; r_center++)
		{
	
			// Add to all except the diagonal
			if (r_center != curKInd)
			{
				// Find the squared L2 dist from the current point to the current k-mean center
				// deltaDist = abs(sum(dot(centers(r_center,0), source_cloud(r_cl)))-dist2home);
			
				// Update adjacency matrix
				// adj_kmeans(r_center, curKInd) += 1/deltaDist;
				// adj_kmeans(curKInd, r_center) = adj_kmeans(r_center, curKInd);
			}
		}
	}
	
	// Normalize density
	
	// Remove below cutoff weight
}

tracker::tracker(int k)
{
    tracker::k = k;
    cluster_ind = cv::Mat(0, 1, CV_32FC1);	// Is float correct?????????
    adj_kmeans = cv::Mat(k, k, CV_32FC1);
    centers = cv::Mat(k, 3, CV_32FC1);
}