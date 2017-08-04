/**
 * Author: Adam Mooers
 *
 * Runs the high-level operations of the pose estimator. This 
 * includes capturing inputs, running clusting, and displaying
 * the result in realtime.
 */

#define PREFILTER_MANHATTAN_DIST 4
#define PREFILTER_DEPTH_MAX_DIST 0.05f
#define KMEANS_K 30
#define KMEANS_ATTEMPTS 1
#define KMEANS_ITERATIONS 20
#define KMEANS_EPSILON 0.002
#define KMEANS_CONNECT_THRESHOLD 0.4
#define CALIBRATION_FILE "calibration.xml"

#include <iostream>
#include <strings.h>
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <GL/glu.h>
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

/**
 * Draws the given pointcloud to the specified window.
 */

void draw_pointcloud(cv::Mat cloud)
{
    glPointSize(2);
    glBegin(GL_POINTS);

    for (int r = 0; r<cloud.rows; r++)
    {
        float* curPoint = cloud.ptr<float>(r);

        glColor3ub(0, (256-(int)(curPoint[1]*512))%256, 0);

        if (curMode == CALIBRATION)
        {
            glVertex3f(curPoint[0], curPoint[1], 0);    // Render x->x, y->y
        }
        else
        {
            glVertex3f(curPoint[0], -curPoint[2], 0);   // Render x->x, -z->y
        }
    }

    glEnd();
}

/**
 * Draws the given adjacency matrix with the corresponding kmeans centers.
 */

void draw_kmeans_mesh(cv::Mat centers, cv::Mat adj)
{
    if (centers.rows != KMEANS_K)
    {
        return;
    }

    glPointSize(6);

    // Draw cloud centers
    glBegin(GL_POINTS);

        for (int r = 0; r<centers.rows; r+=1)
        {
            float* curPoint = centers.ptr<float>(r/1);

            glColor3ub(255, 0, 0);
            glVertex3f(curPoint[0], -curPoint[2], 0); 
        }

    glEnd();

    glBegin(GL_LINES);

        for (int r = 0; r<centers.rows; r+=1)
        {
            float* curPoint = centers.ptr<float>(r);

            for (int c = r; c<adj.cols; c++)
            {
                if (adj.at<float>(r,c) > 0.5f)
                {
                    float* connectedTo = centers.ptr<float>(c);
                    glColor3ub(255, 0, 0);
                    glVertex3f(curPoint[0], -curPoint[2], 0);
                    glVertex3f(connectedTo[0], -connectedTo[2], 0);
                }
            }
        }
    glEnd();
}

int main(int argc, char* argv[])
{
    parse_input(argc, argv);

    float scale_size = curMode == CALIBRATION?0.2:0.15;

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

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // Match coordinates between SFML view with the orthographic view
    sf::Vector2f viewSize = window.getView().getSize();
    sf::Vector2f viewCenter = window.getView().getCenter();
    gluOrtho2D( viewCenter.x-viewSize.x/2,
                viewCenter.x+viewSize.x/2,
                viewCenter.y+viewSize.y/2,
                viewCenter.y-viewSize.y/2);

    // run the main loop
    bool running = true;
    while (running)
    {
        // Update window view
        window.clear(sf::Color::Black);

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
            bool couldCluster = tracker_top.cluster(KMEANS_ATTEMPTS, KMEANS_ITERATIONS, KMEANS_EPSILON);

            if (couldCluster)
            {
                tracker_top.connect_means(KMEANS_CONNECT_THRESHOLD);
                draw_kmeans_mesh(tracker_top.centers, tracker_top.adj_kmeans);
            }
        }

        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                running = false;    // end the program
            }
        }
        
        draw_pointcloud(cam_top.cloud.cloud_array);

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