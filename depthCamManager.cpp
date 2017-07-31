/**
 * Author: Adam Mooers
 *
 * Implements the libraries found in depthCamManager.h.
 */

#include "depthCamManager.h"
#include <cstdio>
#include <list>

bool depth_cam::depth_cam_init() try
{
    if (ctx == nullptr)
    {
        ctx = new rs::context();
    }

    // Are any realsense devices connected?
    if(ctx->get_device_count() == 0) 
    {
        throw std::runtime_error( "No realsense devices are connected to the system at this time." );
    }

    dev = ctx->get_device(0);

    // Configure the input stream
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);

    return true;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());

    dev = nullptr;
    return false;
}

void depth_cam::start_stream( void )
{
    if (dev) {
        dev->start();
    }
}


void depth_cam::capture_next_frame( void )
{
    // Use polling to capture the next frame
    dev->wait_for_frames();

    // Update depth frame meta info
    depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);

    // Retrieve a reference to the raw depth frame
    srcImg = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

    // Convert to an OpenCV style matrix for consistency
    cv::Mat sourceInMatForm(depth_intrin.height, depth_intrin.width, CV_16UC1, (void *)srcImg);

    // Make a copy of the depth frame for editing
    sourceInMatForm.copyTo(cur_src);
}

 void depth_cam::filter_background( float maxDist, int manhattan )
 {
    // Info about the largest index
    int largest_ind = -1;
    int largest_ind_area = 0;

    int current_ind = 0;

    uint16_t* p;

    cv::Mat clustered = cv::Mat::zeros(cur_src.rows, cur_src.cols, CV_32SC1);

    for( int i = 0; i < cur_src.rows; ++i)
    {
        p = cur_src.ptr<uint16_t>(i);
        for ( int j = 0; j < cur_src.cols; ++j)
        {
            if (p[j] != 0)  // For each non-zero pixel
            {
                int current_ind_area = img_BFS(j, i, current_ind, cur_src, clustered, maxDist, 2);

                // Is the new cluster bigger
                if (current_ind_area > largest_ind_area)
                {
                    largest_ind_area = current_ind_area;
                    largest_ind = current_ind;
                }
                current_ind++;
            }
        }
    }

    printf("Current ind: %d\n", current_ind);
 }

int depth_cam::img_BFS( int x, int y, int cluster_id, cv::Mat& input_img, cv::Mat& cluster_img, float maxDist, int manhattan)
{
    float scale = dev->get_depth_scale();
    int cluster_area  = 1;

    std::list<cv::Vec2i> neighbors;

    while (true) 
    {
        cluster_area++;

        // Iterate through all squares within the manhattan distance of the origin
        for (int y_ind = -manhattan + y; y_ind <= manhattan + y; y_ind++)
        {
            uint16_t* in_p = input_img.ptr<uint16_t>(y_ind);    // Pointer to the current row
            int32_t* clust_p = input_img.ptr<int32_t>(y_ind);

            int dxLim = std::abs(y_ind-y)-manhattan;
            for (int x_ind = dxLim + x; x_ind <= -dxLim + x; x_ind++)
            {
                // Skip any out-of-range pixels
                if (in_p[x_ind] == 0 || in_p[x_ind]*scale > maxDist )
                {
                    continue;
                }

                // Add the current pixel to the list
                neighbors.push_back(cv::Vec2i(x_ind, y_ind));

                // Add current item to the visited list
                in_p[x_ind] = 0;

                // Set group of current pixel
                clust_p[x_ind] = cluster_id;
            }
        }

        // Get the next item to evaluate, while more items exist
        if (!neighbors.empty())
        {
            x = neighbors.front()[1];
            y = neighbors.front()[2];
            neighbors.pop_front();
        }
        else
        {
            break;
        }

    }

    return cluster_area;
}

depth_cam::depth_cam( void )
{
    // Only display warnings (avoid verbosity)
    rs::log_to_console(rs::log_severity::warn);
}

depth_cam::~depth_cam( void )
{
    if (ctx != nullptr)
    {
        delete ctx;
    }
}