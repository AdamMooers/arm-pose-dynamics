/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#include "depthCamManager.h"
#include "opencv2/highgui/highgui.hpp"

int main()
{
    depth_cam camTop;
    camTop.depth_cam_init();    // Connect to the depth camera
    camTop.start_stream();

    std::string window_name = "depth feed";
    // Create a window
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE );

    for(;;)
    {
        camTop.capture_next_frame();

        cv::imshow( window_name, camTop.modifiedSrc );

        // Break if a key is pressed
        if (cv::waitKey(1) != -1)
        {
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}