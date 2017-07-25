// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <cstdio>

// SFML Setup
#include <SFML/Window.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Graphics.hpp>
#include <GL/glu.h>

sf::View graphView(sf::FloatRect(-1.5, 0, 3, 1.5));

int abs(int val)
{
	return val<0?-val:val;
}

int main() try
{
    rs::log_to_console(rs::log_severity::warn);

    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    rs::device * dev = ctx.get_device(0);

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->start();

    // create the window
    sf::RenderWindow window(sf::VideoMode(800, 600), "OpenGL", sf::Style::Default, sf::ContextSettings(24));
    window.setVerticalSyncEnabled(true);
    window.setActive(true);
    window.setView(graphView);

    // load resources, initialize the OpenGL states, ...
    sf::Clock clock;

    sf::Font guiFont;
    guiFont.loadFromFile("UbuntuMono-B.ttf");

    // Create FPS test
    sf::Text text("", guiFont);
    text.setCharacterSize(30);
    text.setStyle(sf::Text::Bold);
    text.setColor(sf::Color::Red);
    text.setScale(0.002f, 0.002f);
    text.setPosition(0,0);

    glClearColor(0,0,0,0);

    int countDownMs = 0;    // Countdown in milliseconds until next save
    int elapsedTime = 0;
    bool saveNow = false;

    // run the main loop
    bool running = true;
    while (running)
    {
        sf::Time deltaT = clock.restart();
	    dev->wait_for_frames();

        // handle events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                // end the program
                running = false;
            }
            else if (event.type == sf::Event::Resized)
            {
                // adjust the viewport when the window is resized
                glViewport(0, 0, event.size.width, event.size.height);
            }
            else if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Space)
                {
                    countDownMs = 3000; // Time until the next save occurs
                }
            }
        }

        // Retrieve the depth image
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

        // Retrieve camera parameters for collecting the depth
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        float scale = dev->get_depth_scale();

        rs::float2 camPos = {0, 1};

        // Set up a perspective transform in a space
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        // Match coordinates between SFML view with the orthographic view
        sf::Vector2f viewSize = window.getView().getSize();
        sf::Vector2f viewCenter = window.getView().getCenter();
        gluOrtho2D( viewCenter.x-viewSize.x/2,
                    viewCenter.x+viewSize.x/2,
                    viewCenter.y+viewSize.y/2,
                    viewCenter.y-viewSize.y/2);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        // Render depth data in 3D
        glPointSize(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
        for(int dy=1; dy<depth_intrin.height-1; dy+=1)
        {
            for(int dx=1; dx<depth_intrin.width-1; dx+=1)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;

                // Skip over pixels with a depth value of zero, which is used to indicate no data
                if(depth_value == 0) continue;

                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);

                //if (depth_in_meters > (elapsedTime%4000)/4000.f &&
                //    depth_in_meters < (elapsedTime%4000)/4000.f+0.1f)
               // {
                    // Set point render color
                    glColor3ub((depth_value/2)%4000, 0, 0);

                    // Emit a vertex at the 3D location of this depth pixel
                    glVertex3f(depth_point.x, depth_point.y+0.5, 0);
                //}

                if (saveNow)
                {
                    printf("%f, %f, %f\n", depth_point.x, depth_point.y, depth_point.z);
                }
            }
        }
	    glEnd();

        window.resetGLStates();
        text.setString("FPS: "+std::to_string(1000.0f/deltaT.asMilliseconds()));
        window.draw(text);

        // end the current frame (internally swaps the front and back buffers)
        window.display();

        // Update countdown timer

        // Trigger countdown over?
        saveNow = countDownMs > 0 && (countDownMs-deltaT.asMilliseconds() <= 0);
        elapsedTime += deltaT.asMilliseconds();
        countDownMs -= deltaT.asMilliseconds();
    }

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
