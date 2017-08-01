/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#define PREFILTER_MANHATTAN_DIST 4
#define PREFILTER_DEPTH_MAX_DIST 0.05f

#include <strings.h>
#include "depthCamManager.h"
#include "opencv2/highgui/highgui.hpp"

enum opModes {TRACKING, CALIBRATION};

opModes curMode;

/**
 * Parses the user input. Handles errors such as incorrect argument count, etc.
 */
void parse_input(int argc, char* argv[]) 
{
    if (argc > 2)
    {
        printf("Correct Usage: %s [calibrate]\n", argv[0]);
        exit(0);
    }

    if (argc == 2 && strcmp(argv[1], "calibrate") == 0)
    {
        printf("Entering calibration mode...\n");
        curMode = CALIBRATION;
    }
    else
    {
        printf("Entering tracking mode...\n");
        curMode = TRACKING;
    }
}

int main(int argc, char* argv[])
{
    parse_input(argc, argv);

    float scale_size = curMode == CALIBRATION?0.2:0.5;

    depth_cam cam_top(scale_size);
    cam_top.depth_cam_init();    // Connect to the depth camera
    cam_top.start_stream();

    std::string window_name = "depth feed";
    // Create a window
    cv::namedWindow(window_name, CV_WINDOW_NORMAL );
    cv::resizeWindow(window_name, 640, 480);

    for(;;)
    {
        cam_top.capture_next_frame();
        cam_top.filter_background(PREFILTER_DEPTH_MAX_DIST, PREFILTER_MANHATTAN_DIST);
        cam_top.to_depth_frame();

        if (curMode == CALIBRATION)
        {
            cam_top.cloud.get_transform_from_cloud();
        }
        
        cv::imshow(window_name, cam_top.cur_src);

        // Break if a key is pressed
        if (cv::waitKey(1) != -1)
        {
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}