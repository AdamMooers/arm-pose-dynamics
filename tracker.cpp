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

void tracker::connect_means(float threshold)
{
	cv::Mat k_histogram = cv::Mat::zeros(k, 1, CV_32FC1);
	
	// Clear the matrix
	adj_kmeans = adj_kmeans*0;
	
	// for each point in the cloud
	for (int r_cl=0; r_cl<source_cloud.rows; r_cl++)
	{
		int32_t curKInd = cluster_ind.at<int32_t>(r_cl,0);
		
		// Calculate distance to the center of the home cluster
		float homeDist = cv::norm(centers.row(curKInd), source_cloud.row(r_cl));
		
		// Update histogram for normalization
		k_histogram.at<float>(curKInd,0) += 1;
		
		// for each k-mean
		for(int r_center=0; r_center<centers.rows; r_center++)
		{
	
			// Add to all except the diagonal
			if (r_center != curKInd)
			{
				// Find the L2 dist from the current point to the current k-mean center,
				float deltaDist = cv::norm(centers.row(r_center), source_cloud.row(r_cl));

				// How much closer is the pint to its own cluster than the current one?
				deltaDist = (float)fabs(deltaDist-homeDist);
			
				// Update adjacency matrix
				adj_kmeans.at<float>(r_center, curKInd) += 1/deltaDist;

				// Graph is not directed
				adj_kmeans.at<float>(curKInd, r_center) = adj_kmeans.at<float>(r_center, curKInd);
			}
		}
	}

	// Normalize density
	for (int j=0; j<adj_kmeans.rows; j++)
	{
		adj_kmeans.col(j) /= k_histogram;
		adj_kmeans.row(j) /= k_histogram.t();
	}
	
	// Remove below cutoff weight
	for( int row = 0; row < adj_kmeans.rows-1; ++row)
	{
		for( int col = row+1; col < adj_kmeans.cols; ++col)
		{
			adj_kmeans.at<float>(row, col) = adj_kmeans.at<float>(row, col)>threshold? 1.f:0.f;
		}
	}

	// Make the matrix symmetrical after the thesholding
	cv::completeSymm(adj_kmeans);
}

tracker::tracker(int k)
{
    tracker::k = k;
    cluster_ind = cv::Mat(0, 1, CV_32SC1);
    adj_kmeans = cv::Mat(k, k, CV_32FC1);
    centers = cv::Mat(k, 3, CV_32FC1);
}

bool arm::update_arm_list(float max_dist_to_start, float dx_threshold)
{
	kmean_ind.clear();

}

int arm::find_closest_center_hand()
{
	int closest_ind = -1;
	float closest_dist = FLT_MAX;

	// For each row in the kmeans
	for (int r_c=0; r_c<source->centers.rows; r_c++)
	{
		// Make sure the new point has a greater Z
		if (start_pos.at<float>(0,2) <= source->centers.at<float>(0,2))
		{
			float deltaDist = cv::norm(source->centers.row(r_c), start_pos);

			if (deltaDist < closest_dist)
			{
				closest_ind = r_c;
				closest_dist = deltaDist;
			}
		}
	}

	return closest_ind;
}

arm::arm(tracker& source, cv::Mat start_pos)
{
	arm::start_pos = start_pos;
	arm::source = &source;
}