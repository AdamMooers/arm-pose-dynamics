/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#define PREFILTER_MANHATTAN_DIST 4
#define PREFILTER_DEPTH_MAX_DIST 0.05f
#define KMEANS_K 20
#define KMEANS_ATTEMPTS 2
#define KMEANS_ITERATIONS 20
#define KMEANS_EPSILON 0.01
#define CALIBRATION_FILE "calibration.xml"

#include <strings.h>
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include "depthCamManager.h"
#include "tracker.h"

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

    float scale_size = curMode == CALIBRATION?0.2:0.2;

    depth_cam cam_top(scale_size);
    tracker tracker_top(KMEANS_K);

    cam_top.depth_cam_init();    // Connect to the depth camera
    cam_top.start_stream();

    if (curMode == TRACKING)
    {
        cam_top.cloud.load_calibration_matrix(CALIBRATION_FILE);
    }

    // Create a window
    sf::RenderWindow window(sf::VideoMode(800, 600), "OpenGL", sf::Style::Default, sf::ContextSettings(24));
    sf::View graphView(sf::FloatRect(-0.5, -0.75, 1, 0.75));
    window.setVerticalSyncEnabled(true);
    window.setActive(true);
    window.setView(graphView);

    // run the main loop
    bool running = true;
    while (running)
    {
        cam_top.capture_next_frame();
        cam_top.filter_background(PREFILTER_DEPTH_MAX_DIST, PREFILTER_MANHATTAN_DIST);
        cam_top.to_depth_frame();

        // Convert to point cloud and apply calibration transform to it
        if (curMode == CALIBRATION)
        {
            cam_top.cloud.get_transform_from_cloud();
        }

        cam_top.cloud.transform_cloud();

        if (curMode == TRACKING)
        {
            // Run clustering algorithm
            tracker_top.update_point_cloud(cam_top.cloud);
            tracker_top.cluster(KMEANS_ATTEMPTS, KMEANS_ITERATIONS, KMEANS_EPSILON);
        }

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                running = false;    // end the program
            }
        }
        
        // Update window view
        window.clear(sf::Color::Black);

        sf::CircleShape screendot(0.005, 3);
        screendot.setFillColor(sf::Color(100, 250, 50));
        screendot.setOrigin(screendot.getRadius(), screendot.getRadius());

        cv::Mat cloud = cam_top.cloud.cloud_array;

        for (int r = 0; r<cloud.rows; r++)
        {
            float* curPoint = cloud.ptr<float>(r);

            if (curMode == CALIBRATION)
            {
                screendot.setPosition(curPoint[0], curPoint[1]);    // Render x->x, y->y
            }
            else
            {
                screendot.setPosition(curPoint[0], -curPoint[2]);    // Render x->x, -z->y
            }
            window.draw(screendot);
        }

        // Render in screen space
        screendot.setRadius(0.01);
        screendot.setFillColor(sf::Color(200, 0, 0));
        cloud = tracker_top.centers;
        for (int r = 0; r<cloud.rows; r++)
        {
            float* curPoint = cloud.ptr<float>(r);
            screendot.setPosition(curPoint[0], -curPoint[2]);    // Render x->x, -z->y
            window.draw(screendot);
        }

        window.display();
    }

    window.close();

    // Get transform from cloud
    if (curMode == CALIBRATION)
    {
        cam_top.cloud.prompt_for_manual_offset();
        printf("Saving calibration transform to " CALIBRATION_FILE "...\n");
        cam_top.cloud.save_calibration_matrix(CALIBRATION_FILE);
    }

    return 0;
}