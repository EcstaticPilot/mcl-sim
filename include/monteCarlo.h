
#pragma once
#include <vector>
#include <random>
#include "odometry.h"
#include <SDL2/SDL.h>
#include <utility>
extern const double SCALE_FACTOR;

class pose : public point
{
public:
    float theta;
    pose(float x, float y, float theta) : point(x, y), theta(theta) {}
    pose(point pt, float theta) : point(pt), theta(theta) {}
    pose() : point(0, 0), theta(0) {}
    void print();
};
class sample : public pose
{
public:
    float w;
    sample(float x, float y, float theta, float w) : pose(x, y, theta), w(w) {}
    sample(pose p1, float w) : pose(p1), w(w) {}
    sample() : pose(0, 0, 0), w(0) {}
};
namespace MonteCarlo
{

    // samples
    extern std::vector<sample> samples;

    // mersenne twister random number generator
    extern std::mt19937 randgen;

    // max samples
    extern uint J;

    // odom stuff
    extern double prevLE; // create previous encoder value left
    extern double prevRE; // create previous encoder value right
    extern double prevBE; // create previous encoder value right
    extern double prevAngle;
    extern float spread;
    extern pose prevRobot;
    extern pose MCLRobot;
    const pose backoffset = pose(-4, -7.5, 180);
    const pose leftoffset = pose(-4.5, 4, -90);
    const pose rightoffset = pose(4, -3, 90);
    const pose frontoffset = pose(5.5, 6, 0);
    void init(pose start, int max_samples);
    float maxWeight();
    void resample();
    void update();
    
    void relocalize();
    float distanceSensorWeight(sample particle, float d, pose offset);
    void normalizeSamples();

    void moveCenter(point c);
    float meanWeight();
    pose getPose();
    void thread();
    float getESS();
    void setParticleCount(int count);
    float getStdDev();
    void renderDistanceSensors(SDL_Renderer *renderer);
    void renderParticles(SDL_Renderer *renderer);
    void renderRobot(SDL_Renderer *renderer);
};