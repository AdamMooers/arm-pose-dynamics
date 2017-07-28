/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#include "depthCamManager.h"

int main()
{
    depth_cam camTop;
    camTop.depth_cam_init();    // Connect to the depth camera
    camTop.start_stream();

    getchar();

    return 0;
}