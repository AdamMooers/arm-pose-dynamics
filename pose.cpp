/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#define PREFILTER_MANHATTAN_DIST 4
#define PREFILTER_DEPTH_MAX_DIST 0.05f
#define CALIBRATION_FILE "calibration.xml"

#include <strings.h>
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include "depthCamManager.h"

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

    if (curMode == TRACKING)
    {
        cam_top.cloud.load_calibration_matrix(CALIBRATION_FILE);
    }

    // Create a window
    sf::RenderWindow window(sf::VideoMode(800, 600), "OpenGL", sf::Style::Default, sf::ContextSettings(24));
    sf::View graphView(sf::FloatRect(-1, -0.75, 2, 1.5));
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

       // sf::VertexArray triangle(sf::Triangles, 3);
        sf::CircleShape screendot(0.01, 4);
        screendot.setFillColor(sf::Color(100, 250, 50));
        screendot.setOrigin(screendot.getRadius(), screendot.getRadius());

        cv::Mat cloud = cam_top.cloud.cloud_array;

        for (int r = 0; r<cloud.rows; r++)
        {
            float* curPoint = cloud.ptr<float>(r);
            screendot.setPosition(curPoint[0], curPoint[1]);
            window.draw(screendot);
        }
        window.display();
    }

    // Get transform from cloud
    if (curMode == CALIBRATION)
    {
        printf("Saving calibration transform to " CALIBRATION_FILE "...\n");
        cam_top.cloud.save_calibration_matrix(CALIBRATION_FILE);
    }

    return 0;
}