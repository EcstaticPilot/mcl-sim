#include <SDL2/SDL.h>
#include <iostream>
#include "monteCarlo.h"
// Constants for the screen size
const int SCREEN_WIDTH = 1000;  // Window width
const int SCREEN_HEIGHT = 1000; // Window height

// Scale factor to convert inches to pixels (adjust as needed)
extern const double SCALE_FACTOR;
const double SCALE_FACTOR = 5.0; // 1 inch = 5 pixels
float remap(float value, float a, float b, float c, float d);
void drawRobot(SDL_Renderer *renderer, SDL_Texture *texture, int x, int y, int size, double angle)
{
    SDL_Point center = {size / 2, size / 2};
    SDL_Rect robot = {x - size / 2, y - size / 2, size, size};

    SDL_RenderCopyEx(renderer, texture, NULL, &robot, angle, &center, SDL_FLIP_NONE);
}
void drawLine(SDL_Renderer *renderer, int x1, int y1, int x2, int y2)
{
    //! HARD CODED BAD
     int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
    int x = (SCREEN_WIDTH - boxSize) / 2; // Center the box
    int y = (SCREEN_HEIGHT - boxSize) / 2;
    x1 = x1*SCALE_FACTOR + (x + boxSize / 2);
    y1 = y1*SCALE_FACTOR + (y + boxSize / 2);
    x2 = x2*SCALE_FACTOR + (x + boxSize / 2);
    y2 = y2*SCALE_FACTOR + (y + boxSize / 2);
    SDL_RenderDrawLineF(renderer,x1,y1,x2,y2);
}
void drawCircle(SDL_Renderer *renderer, int centreX, int centreY, float r)
{
    const int32_t diameter = (r * 2);

    int32_t x = (r - 1);
    int32_t y = 0;
    int32_t tx = 1;
    int32_t ty = 1;
    int32_t error = (tx - diameter);

    while (x >= y)
    {
        //  Each of the following renders an octant of the circle
        SDL_RenderDrawPoint(renderer, centreX + x, centreY - y);
        SDL_RenderDrawPoint(renderer, centreX + x, centreY + y);
        SDL_RenderDrawPoint(renderer, centreX - x, centreY - y);
        SDL_RenderDrawPoint(renderer, centreX - x, centreY + y);
        SDL_RenderDrawPoint(renderer, centreX + y, centreY - x);
        SDL_RenderDrawPoint(renderer, centreX + y, centreY + x);
        SDL_RenderDrawPoint(renderer, centreX - y, centreY - x);
        SDL_RenderDrawPoint(renderer, centreX - y, centreY + x);

        if (error <= 0)
        {
            ++y;
            error += ty;
            ty += 2;
        }

        if (error > 0)
        {
            --x;
            tx += 2;
            error += (tx - diameter);
        }
    }
}
void drawLadder(SDL_Renderer *renderer, int x, int y)
{
    SDL_Point points[] = {
        {x, y + static_cast<int>(SCALE_FACTOR)}, {x - static_cast<int>(SCALE_FACTOR), y}, {x - static_cast<int>(SCALE_FACTOR), y}, {x, y - static_cast<int>(SCALE_FACTOR)}, {x, y - static_cast<int>(SCALE_FACTOR)}, {x + static_cast<int>(SCALE_FACTOR), y}, {x + static_cast<int>(SCALE_FACTOR), y}, {x, y + static_cast<int>(SCALE_FACTOR)}};

    SDL_RenderDrawLines(renderer, points, 8);
}

void drawGrid(SDL_Renderer *renderer, int x, int y, int boxSize, int gridSize)
{
    int gridSizePixels = static_cast<int>(gridSize * SCALE_FACTOR);
    int centerX = x + boxSize / 2;
    int centerY = y + boxSize / 2;

    // Draw vertical lines
    for (int i = centerX; i <= x + boxSize; i += gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, i, y, i, y + boxSize);
    }
    for (int i = centerX; i >= x; i -= gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, i, y, i, y + boxSize);
    }

    // Draw horizontal lines
    for (int i = centerY; i <= y + boxSize; i += gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, x, i, x + boxSize, i);
    }
    for (int i = centerY; i >= y; i -= gridSizePixels)
    {
        SDL_RenderDrawLine(renderer, x, i, x + boxSize, i);
    }
}

int main(int argc, char *argv[])
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0)
    {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow("MCL sim",
                                          SDL_WINDOWPOS_CENTERED,
                                          SDL_WINDOWPOS_CENTERED,
                                          SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (!window)
    {
        std::cerr << "SDL_CreateWindow Error: " << SDL_GetError() << std::endl;
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer)
    {
        std::cerr << "SDL_CreateRenderer Error: " << SDL_GetError() << std::endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    // robot
    SDL_Texture *robotTexture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR);
    if (!robotTexture)
    {
        std::cerr << "SDL_CreateTexture Error: " << SDL_GetError() << std::endl;
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Set the robot texture to a solid color (e.g., red)
    SDL_SetTextureBlendMode(robotTexture, SDL_BLENDMODE_BLEND);

    SDL_SetRenderTarget(renderer, robotTexture);
    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 55);
    SDL_RenderClear(renderer);
    SDL_Rect rect = {0, 0, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR};

    // Draw arrow
    // Draw an arrow centered in the texture
    int centerX = 24 * SCALE_FACTOR / 2;
    int centerY = 24 * SCALE_FACTOR / 2;
    int arrowLength = 12 * SCALE_FACTOR;
    int arrowHeadSize = 5 * SCALE_FACTOR;
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
    // Arrow shaft
    SDL_RenderDrawLine(renderer, centerX, centerY, centerX, centerY - arrowLength);

    // Arrowhead
    SDL_RenderDrawLine(renderer, centerX, centerY - arrowLength, centerX - arrowHeadSize, centerY - arrowLength + arrowHeadSize);
    SDL_RenderDrawLine(renderer, centerX, centerY - arrowLength, centerX + arrowHeadSize, centerY - arrowLength + arrowHeadSize);

    SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
    SDL_RenderDrawRect(renderer, &rect); // Draw the outline of the rectangle

    SDL_SetRenderTarget(renderer, NULL);

    SDL_Texture *robotTexture2 = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_TARGET, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR);
    SDL_SetTextureBlendMode(robotTexture2, SDL_BLENDMODE_BLEND);
    SDL_SetRenderTarget(renderer, robotTexture2);
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 50);
    SDL_RenderClear(renderer);
    SDL_Rect rect2 = {0, 0, 24 * SCALE_FACTOR, 24 * SCALE_FACTOR};
    SDL_SetRenderDrawColor(renderer, 255, 0, 255, 255);
    SDL_RenderDrawRect(renderer, &rect2); // Draw the outline of the rectangle
    SDL_SetRenderTarget(renderer, NULL);

    // Box dimensions (scaled to pixels)
    int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
    int x = (SCREEN_WIDTH - boxSize) / 2; // Center the box
    int y = (SCREEN_HEIGHT - boxSize) / 2;

    // Main loop
    bool running = true;
    SDL_Event event;
    // Clear the screen with a white background
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_RenderClear(renderer);

    // Set the draw color to black
    SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

    // Draw the hollow box
    SDL_Rect box = {x, y, boxSize, boxSize};
    SDL_RenderDrawRect(renderer, &box);
    // drawLadder(renderer, 24 * SCALE_FACTOR, 0);
    //  Present the renderer
    SDL_RenderPresent(renderer);
    Robot = pose(0, 0, 0);
    bool moveForward = false, moveBackward = false, turnLeft = false, turnRight = false;

    const int FRAME_RATE = 30;
    const int FRAME_DELAY = 1000 / FRAME_RATE;
    std::cout << "Starting MCL" << std::endl;
    int J = 300;
    MonteCarlo::init(pose(0, 0, 0), 300);
    // SDL_Delay(500);
    MonteCarlo::renderParticles(renderer);
    std::cout << "MCL started" << std::endl;
    pose MCLpose = {0, 0, 0};
    SDL_RenderPresent(renderer);
    pose prevRobot;
    // SDL_Delay(5000);
    float a = 0;
    while (running)
    {
        Uint32 frameStart = SDL_GetTicks();
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_w:
                    moveForward = true;
                    break;
                case SDLK_s:
                    moveBackward = true;
                    break;
                case SDLK_a:
                    turnLeft = true;
                    break;
                case SDLK_d:
                    turnRight = true;
                    break;
                }
            }
            else if (event.type == SDL_KEYUP)
            {
                switch (event.key.keysym.sym)
                {
                case SDLK_w:
                    moveForward = false;
                    break;
                case SDLK_s:
                    moveBackward = false;
                    break;
                case SDLK_a:
                    turnLeft = false;
                    break;
                case SDLK_d:
                    turnRight = false;
                    break;
                case SDLK_t:
                    MonteCarlo::setParticleCount(500);
                    break;
                case SDLK_g:
                    MonteCarlo::setParticleCount(20);
                    break;
                }
            }
            if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                int x, y;
                SDL_GetMouseState(&x, &y);

                // convert to inches
                x = (x - SCREEN_WIDTH / 2) / SCALE_FACTOR;
                y = -1 * (y - SCREEN_HEIGHT / 2) / SCALE_FACTOR;
                std::cout << "x: " << x << " y: " << y << std::endl;
                MonteCarlo::moveCenter({x, y});
            }
        }

        // Clear the screen with a white background
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderClear(renderer);
        if (turnLeft)
        {
            Robot.theta -= 3;
        }
        if (turnRight)
        {
            Robot.theta += 3;
        }
        if (moveForward)
        {
            Robot.x += (0.35 * SCALE_FACTOR * sin(Robot.theta * M_PI / 180.0));
            Robot.y += (0.35 * SCALE_FACTOR * cos(Robot.theta * M_PI / 180.0));
        }
        if (moveBackward)
        {
            Robot.x -= (0.35 * SCALE_FACTOR * sin(Robot.theta * M_PI / 180.0));
            Robot.y -= (0.35 * SCALE_FACTOR * cos(Robot.theta * M_PI / 180.0));
        }

        // Set the draw color to black
        

        // Draw the hollow box
        SDL_Rect box = {x, y, boxSize, boxSize};
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawRect(renderer, &box);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 25);
        drawGrid(renderer, x, y, boxSize, 24);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawLine(renderer, (6 * SCALE_FACTOR) + (x + boxSize / 2), (9 * SCALE_FACTOR) + (y + boxSize / 2), (-9 * SCALE_FACTOR) + (x + boxSize / 2), (-6 * SCALE_FACTOR) + (y + boxSize / 2));
        SDL_RenderDrawLine(renderer, (9 * SCALE_FACTOR) + (x + boxSize / 2), (6 * SCALE_FACTOR) + (y + boxSize / 2), (-6 * SCALE_FACTOR) + (x + boxSize / 2), (-9 * SCALE_FACTOR) + (y + boxSize / 2));
        drawCircle(renderer, -67 * SCALE_FACTOR + (x + boxSize / 2), (47 * SCALE_FACTOR) + (y + boxSize / 2), 2 * SCALE_FACTOR);
        drawCircle(renderer, 67 * SCALE_FACTOR + (x + boxSize / 2), (47 * SCALE_FACTOR) + (y + boxSize / 2), 2 * SCALE_FACTOR);
        drawCircle(renderer, -67 * SCALE_FACTOR + (x + boxSize / 2), (-47 * SCALE_FACTOR) + (y + boxSize / 2), 2 * SCALE_FACTOR);
        drawCircle(renderer, 67 * SCALE_FACTOR + (x + boxSize / 2), (-47 * SCALE_FACTOR) + (y + boxSize / 2), 2 * SCALE_FACTOR);

        drawLine(renderer,(-24+1.5),(-48-1.5),(-24+3), (-48));
        drawLine(renderer,(-24+1.5),(-48+1.5),(-24+3), (-48));

        drawLine(renderer,-(-24+1.5),(-48-1.5),-(-24+3), (-48));
        drawLine(renderer,-(-24+1.5),(-48+1.5),-(-24+3), (-48));

        drawLine(renderer,(-24+1.5),-(-48-1.5),(-24+3), -(-48));
        drawLine(renderer,(-24+1.5),-(-48+1.5),(-24+3), -(-48));

        drawLine(renderer,-(-24+1.5),-(-48-1.5),-(-24+3), -(-48));
        drawLine(renderer,-(-24+1.5),-(-48+1.5),-(-24+3), -(-48));        
        // draw robot
        drawRobot(
            renderer,
            robotTexture,
            (x + boxSize / 2) + Robot.x * SCALE_FACTOR,
            (y + boxSize / 2) + -Robot.y * SCALE_FACTOR,
            16 * SCALE_FACTOR,
            Robot.theta);

        MonteCarlo::renderDistanceSensors(renderer);
        MonteCarlo::update();

        if (MonteCarlo::meanWeight() * J < 1 * pow(10, -10) || fabs(MCLpose.x) > 70 || fabs(MCLpose.y) > 70)
        {
            std::cout << "kidnapped: " << MonteCarlo::meanWeight() << std::endl;
            MonteCarlo::spread *= 2;
        }
        else
        {
            MonteCarlo::spread = 1;
        }

        MonteCarlo::normalizeSamples();
        MonteCarlo::renderParticles(renderer);
        MCLpose = MonteCarlo::getPose();
        // std::cout << MonteCarlo::getESS()/MonteCarlo::J << std::endl;
        //MonteCarlo::resample();
        double stddev = MonteCarlo::getStdDev();
        if (MonteCarlo::getESS() < (MonteCarlo::J) / 1)
        {
            std::cout << "resampled11111111111" << std::endl;
            MonteCarlo::resample();
        }
        else
        {
        std::cout << "skipped" << std::endl;
        }

        drawRobot(
            renderer,
            robotTexture2,
            (x + boxSize / 2) + (((MCLpose.x)) * SCALE_FACTOR),
            (y + boxSize / 2) - (((MCLpose.y)) * SCALE_FACTOR),
            16 * SCALE_FACTOR,
            MCLpose.theta);
        prevRobot = MCLpose;
        // MonteCarlo::getPose().print();
        //  Present the renderer
        SDL_RenderPresent(renderer);
        Uint32 frameTime = SDL_GetTicks() - frameStart;
        if (frameTime < FRAME_DELAY)
        {
            SDL_Delay(FRAME_DELAY - frameTime);
        }
    }

    // Cleanup
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
