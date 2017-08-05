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

bool arm::update_arm_list()
{
	kmean_ind.clear();
	int hand_ind = find_closest_center_hand();

	// Was the hand_index found?
	if (hand_ind == -1)
	{
		return false;
	}

	cv::Mat adj = source->adj_kmeans;
	cv::Mat ctrs = source->centers;

	kmean_ind.push_back(hand_ind);

	float x_last = ctrs.at<float>(hand_ind,0);
	float z_last = ctrs.at<float>(hand_ind,2);
	float orientation = start_pos.at<float>(0,0)>0?-1:1;

	for(int cur_ind = hand_ind;;)
	{
		float furthest_dist = 0;

		// For each hand index
		for (int n_ind=0; n_ind<adj.cols; n_ind++)
		{
			if (adj.at<float>(cur_ind,n_ind) > 0.5 &&					// Is center a neighbor?
				ctrs.at<float>(cur_ind,2) < ctrs.at<float>(n_ind,2))	// is z greater?
			{
				float dist2mean = -orientation*ctrs.at<float>(n_ind,0);
				
				if (dist2mean > furthest_dist)
				{
					furthest_dist = dist2mean;
					cur_ind = n_ind;
				}
			}
		}

		// furthest_dist does not change if no points were added.
		if (furthest_dist == 0)
		{
			break;
		}
		else
		{
			float x_cur = ctrs.at<float>(cur_ind,0);
			float z_cur = ctrs.at<float>(cur_ind,2);
			float dx_dz = (x_cur-x_last)/(z_cur-z_last);

			// Correct slope depending on if arm is left or right
			dx_dz *= orientation;

			if (fabs(dx_dz) >= dxdz_threshold)
			{
				break;
			}

			x_last = x_cur;
			z_last = z_cur;
		}

		// Push to list
		kmean_ind.push_back(cur_ind);
	}

	return kmean_ind.size()>=3?true:false;	// Atleast 3 joints needed to form arm
}

void arm::update_elbow_approx(void)
{
	elbow_approx_ind = -1;
	float dist_mult_max = 0;

	for (std::list<int>::const_iterator ci = kmean_ind.begin(); ci != kmean_ind.end(); ++ci)
    {
		float dist_mult = 1;
		dist_mult *= cv::norm(source->centers.row(*ci), source->centers.row(kmean_ind.front()));
		dist_mult *= cv::norm(source->centers.row(*ci), source->centers.row(kmean_ind.back()));

		if (dist_mult > dist_mult_max)
		{
			dist_mult_max = dist_mult;
			elbow_approx_ind = *ci;
		}
	}
}

int arm::find_closest_center_hand(void)
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

			if (deltaDist < closest_dist && deltaDist < max_dist_to_start)
			{
				closest_ind = r_c;
				closest_dist = deltaDist;
			}
		}
	}

	return closest_ind;
}

bool arm::update_joints(float smoothing_factor)
{
	tracking_step++;

	bool can_track = update_arm_list();
	bool is_tracking = (tracking_step-last_tracked_step) < max_missed_steps;

	if (!can_track)	// No joint data, so don't continue
	{
		return is_tracking;
	}

	update_elbow_approx();
	last_tracked_step = tracking_step;

	cv::Mat next_hand_loc =  source->centers.row(kmean_ind.front());
	cv::Mat next_elbow_loc =  source->centers.row(elbow_approx_ind);
	cv::Mat next_shoulder_loc = source->centers.row(kmean_ind.back());

	if (!is_tracking)
	{
		next_hand_loc.copyTo(hand_loc);
		next_elbow_loc.copyTo(elbow_loc);
		next_shoulder_loc.copyTo(shoulder_loc);
	}
	else
	{
		lerp(next_hand_loc, hand_loc, smoothing_factor);
		lerp(next_elbow_loc, elbow_loc, smoothing_factor);
		lerp(next_shoulder_loc, shoulder_loc, smoothing_factor);
	}

	return true;
}

void arm::lerp(cv::Mat target, cv::Mat& current, float t)
{
	current = current+(current-target)*-t;
}

arm::arm(tracker& source, cv::Mat start_pos, float max_dist_to_start, float dxdz_threshold)
{
	arm::max_dist_to_start = max_dist_to_start;
	arm::start_pos = start_pos;
	arm::source = &source;
	arm::dxdz_threshold = dxdz_threshold;
}