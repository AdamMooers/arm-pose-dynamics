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
    depth_cam camTop(0.4);
    camTop.depth_cam_init();    // Connect to the depth camera
    camTop.start_stream();

    std::string window_name = "depth feed";
    // Create a window
    cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE );

    for(;;)
    {
        camTop.capture_next_frame();
        camTop.filter_background( 0.05f, 3 );
        cv::imshow( window_name, camTop.cur_src );

        // Break if a key is pressed
        if (cv::waitKey(1) != -1)
        {
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}