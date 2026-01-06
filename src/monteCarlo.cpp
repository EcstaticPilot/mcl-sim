
#include <iostream>
#include "monteCarlo.h"
#include <chrono>
#include <random>
#include <odometry.h>
#include <SDL2/SDL.h>
#include <algorithm>
#include <utility>
extern const double SCALE_FACTOR;
using namespace MonteCarlo;

/**
 * @brief gaussian PDF centered at mean = 0, translated up by 1
 *
 * @param x the x value
 * @param a maximum deviation from the mean before the value goes to aproximately 1
 * @param b maximum output from the function at x=mean
 * @param mean optional, defaults to 0
 * @return float weight ranging from 1 to b
 */
float gaussianWeight(float x, float a, float b, float mean = 0)
{
    return (b - .001) * exp(-1 * pow(2 * ((x - mean) / a), 2)) + .001;
}

int sign(float a)
{
    return (a > 0 ? 1 : -1);
}
void pose::print()
{
    std::cout << this->x << ", " << this->y << ", " << this->theta << std::endl;
}
// intiialize variables

float remap(float value, float a, float b, float c, float d)
{
    return c + ((value - a) / (b - a)) * (d - c);
}

float raySegmentIntersect(pose ray, point p1, point p2)
{
    // parametric form for funny

    float thetaRad = (90 - ray.theta) * M_PI / 180.0;
    // Calculate the direction vector of the ray
    float rayDirX = cos(thetaRad);
    float rayDirY = sin(thetaRad);

    float m = rayDirY / (fabs(rayDirX) < 0.0001 ? 0.0001 : rayDirX);
    float b = ray.y - m * ray.x;

    float m2 = (p2.y - p1.y) / (p2.x - p1.x != 0 ? p2.x - p1.x : 0.0001);
    float b2 = p1.y - m2 * p1.x;

    float x = (b2 - b) / (m - m2);
    float y = m * x + b;

    // std::cout<<x<<", "<<y<<std::endl;
    // bounds checks
    if (sign(x - ray.x) != sign(rayDirX))
    {
        return -1000;
    }
    if (sign(y - ray.y) != sign(rayDirY))
    {
        return -1000;
    }
    if (x < fmin(p1.x, p2.x) or x > fmax(p1.x, p2.x))
    {
        return -1000;
    }
    if (y < fmin(p1.y, p2.y) or y > fmax(p1.y, p2.y))
    {
        return -1000;
    }
    return (ray.distance(point(x, y)));
}

float lineCircleIntersect(pose ray, point c, float r)
{
    // Convert angle to radians
    float thetaRad = (90 - ray.theta) * M_PI / 180.0;

    // Calculate the direction vector of the ray
    float rayDirX = cos(thetaRad);
    float rayDirY = sin(thetaRad);

    float m = rayDirY / rayDirX;
    float b = ray.y - m * ray.x;

    float m2 = -1 / m;
    float b2 = c.y - m2 * c.x;

    // calculate intersection point
    float x = (b2 - b) / (m - m2);
    float y = m * x + b;

    float d1 = sqrt(pow(x - c.x, 2) + pow(y - c.y, 2));
    if (d1 > r)
        return -1000;

    if (sign(x - ray.x) != sign(rayDirX) || sign(y - ray.y) != sign(rayDirY))
        return -1000;

    float d2 = sqrt(r * r - d1 * d1);
    point i = point(x + sign(ray.x - x) * fabs(d2 * rayDirX), y + sign(ray.y - y) * fabs(d2 * rayDirY));
    return i.distance(ray);
}

namespace MonteCarlo
{


    //?use std::array?
    std::vector<sample> samples;

    std::mt19937 randgen;
    uint J;
    float spread;
    double prevLE = 0;
    double prevRE = 0;
    double prevBE = 0;
    double prevAngle = 0;
    pose prevRobot = pose();
    pose MCLRobot = pose();
    /**
     * @brief initialize mcl
     *
     * @param start
     * @param max_samples
     */
    void init(pose start, int max_samples)
    {
        std::cout << "init" << std::endl;
        // random number generater
        randgen = std::mt19937{(uint)std::chrono::high_resolution_clock::now().time_since_epoch().count()};

        // nomral distrubution generator for encoder noise

        J = max_samples;
        samples.reserve(J);
        samples.resize(J);
        prevRobot = start;
        std::normal_distribution<float> rdn{0, 5};

        spread = 1;
        for (int i = 0; i < J; i++)
        {
            // std::cout<<i<<std::endl;
            float rx = start.x + rdn(randgen);
            float ry = start.y + rdn(randgen);
            float rt = start.theta + rdn(randgen);
            samples[i] = sample(rx, ry, rt, 1.0f / J);
            // samples[i].print();
        }
    }
    /**
     * @brief perform a raycast to simulate a real distance sensor
     *
     * @param offset
     * @param pos
     * @return float
     */
    float distanceReal(pose offset, pose pos)
    {

        float thetaRad = (-pos.theta) * M_PI / 180;
        float sinTheta = sin(thetaRad);
        float cosTheta = cos(thetaRad);

        point offsetdelta = point(
            offset.x * cosTheta - offset.y * sinTheta,
            offset.x * sinTheta + offset.y * cosTheta);
        // offset.print();
        // offsetdelta.print();
        pos.theta += offset.theta;

        //   convert to standard angle
        thetaRad = (90 - pos.theta) * M_PI / 180;

        // precompute sin and cos
        sinTheta = sin(thetaRad);
        cosTheta = cos(thetaRad);
        // size of field box
        float c = 70;

        // calculate offset

        // offsetdelta.print();
        pos.x += offsetdelta.x;
        pos.y += offsetdelta.y;

        // get thge distances
        float d1 = (sinTheta != 0) ? fabs((sign(sinTheta) * c) - pos.y) / sinTheta : std::numeric_limits<float>::max();
        float d2 = (cosTheta != 0) ? fabs((sign(cosTheta) * c) - pos.x) / cosTheta : std::numeric_limits<float>::max();
        d1 = fabs(d1);
        d2 = fabs(d2);

        float d3 = fabs(raySegmentIntersect(pos, point(9, -6), point(-6, 9)));
        float d4 = fabs(raySegmentIntersect(pos, point(6, -9), point(-9, 6)));

        float d5 = fabs(lineCircleIntersect(pos, point(-67, -47), 2));
        float d6 = fabs(lineCircleIntersect(pos, point(-67, 47), 2));
        float d7 = fabs(lineCircleIntersect(pos, point(67, 47), 2));
        float d8 = fabs(lineCircleIntersect(pos, point(67, -47), 2));

        float d9 = fabs(raySegmentIntersect(pos, point(-24 + 3, -48), point(-24 + 1.5, -48 - 1.5)));
        float d10 = fabs(raySegmentIntersect(pos, point(-24 + 3, -48), point(-24 + 1.5, -48 + 1.5)));

        float d11 = fabs(raySegmentIntersect(pos, point(24 - 3, -48), point(24 - 1.5, -48 - 1.5)));
        float d12 = fabs(raySegmentIntersect(pos, point(24 - 3, -48), point(24 - 1.5, -48 + 1.5)));

        float d13 = fabs(raySegmentIntersect(pos, point(-24 + 3, 48), point(-24 + 1.5, 48 + 1.5)));
        float d14 = fabs(raySegmentIntersect(pos, point(-24 + 3, 48), point(-24 + 1.5, 48 - 1.5)));

        float d15 = fabs(raySegmentIntersect(pos, point(24 - 3, 48), point(24 - 1.5, 48 + 1.5)));
        float d16 = fabs(raySegmentIntersect(pos, point(24 - 3, 48), point(24 - 1.5, 48 - 1.5)));

        // pick the smallest distance
        float df = d1;
        df = fmin(df, d2);
        df = fmin(df, d3);
        df = fmin(df, d4);
        df = fmin(df, d5);
        df = fmin(df, d6);
        df = fmin(df, d7);
        df = fmin(df, d8);
        df = fmin(df, d9);
        df = fmin(df, d10);
        df = fmin(df, d11);
        df = fmin(df, d12);
        df = fmin(df, d13);
        df = fmin(df, d14);
        df = fmin(df, d15);
        df = fmin(df, d16);
        if (df > 78.74)
            df = -1;
        return df;
    }
    /**
     * @brief recover from kidnapping
     *
     * @return float
     */
    void relocalize()
    {
        auto random = std::uniform_real_distribution<float>(-65, 65);
        for (int i = 0; i < J; i++)
        {
            samples[i].w = 1.0f / J;
            samples[i].x = random(randgen);
            samples[i].y = random(randgen);
            samples[i].theta = Robot.theta;
        }
    }
    float maxWeight()
    {
        float max = 0;
        for (int i = 0; i < J; i++)
        {
            if (samples[i].w > max)
                max = samples[i].w;
        }
        return max;
    }
    /**
     * @brief calculate weight based on a distance sensor reading
     *
     * @param particle the particle to be evaluated
     * @param d the distance sensor reading
     * @param offset the offset for the distance sensor, containing x,y, and theta
     * @return float
     */
    float distanceSensorWeight(sample particle, float d, pose offset)
    {
        float df = distanceReal(offset, particle);
        if (fabs(df - d) > 24)
            return .0001;
        if (df == -1 && d == -1)
            return .5;
        float w = gaussianWeight((df - d), 6, 1);
        return w;
    }
    /**
     * @brief update MCL
     *
     */
    void update()
    {
        // // update change in robot
        float angle = Robot.theta;
        float dAngle = angle - prevAngle;

        // get robot pose, add noise
        std::normal_distribution<float> rdn{0, 0.5};
        MCLRobot = getPose();

        point delta = {Robot.x - prevRobot.x, Robot.y - prevRobot.y};
        delta.x += rdn(randgen);
        delta.y += rdn(randgen);
        auto rdn2 = std::normal_distribution<float>(0, 0.5 * spread);

        float speed = sqrt(delta.x * delta.x + delta.y * delta.y);

        // std::cout << "speed: " << speed << std::endl;
        float sinTheta = sin((90 - angle) * M_PI / 180);
        float cosTheta = cos((90 - angle) * M_PI / 180);
        float sindTheta = sin((-dAngle) * M_PI / 180);
        float cosdTheta = cos((-dAngle) * M_PI / 180);
        std::normal_distribution<float> rdn3{1, 0.025};
        float dr = distanceReal(rightoffset, Robot) * rdn3(randgen);
        float dl = distanceReal(leftoffset, Robot) * rdn3(randgen);
        float dc = distanceReal(backoffset, Robot) * rdn3(randgen);
        float df = distanceReal(frontoffset, Robot) * rdn3(randgen);
        auto rdn4 = std::normal_distribution<float>(0, 6);

        bool GPSenable = false;
        if (randgen() % 10 == 1)
        {
            GPSenable = false;
        }
        float gpsx;

        float gpsy;
        // TODO: add gps sabotage
        if (GPSenable)
        {
            gpsx = Robot.x + rdn4(randgen);
            gpsy = Robot.y + rdn4(randgen);
        }
        if (randgen() % 100 == 1)
        {
            //::cout << "fake" << std::endl;
            dr = rand() % 70;
        }
        if (randgen() % 100 == 1)
        {
            // std::cout << "fake" << std::endl;
            dl = rand() % 70;
        }
        if (randgen() % 100 == 1)
        {
            // std::cout << "fake" << std::endl;
            dc = rand() % 70;
        }
        // if (i > 9.0f / 10.0f * J&& GPSenable)
        // {
        //
        // }
        // else
        // {

        for (int i = 0; i <= J / 4.0f && GPSenable; i++)
        {
            pose rPose = {gpsx + rdn4(randgen), gpsy + rdn4(randgen), angle};
            sample rSample = sample(
                rPose.x,
                rPose.y,
                angle,
                1.0f / J);
            samples[randgen() % J] = rSample;
        }

        for (int i = 0; i < J; i++)
        {

            // generate random poses
            sample start = samples[i];

            // // generate random angle delta

            // get pose from random values

            // pose rPose = Odometry::odomStep(rLE, rRE, rBE, rAngle, rDAngle);

            // gps generate
            pose rPose;
            // &&

            rPose = {start.x + delta.x + speed * rdn2(randgen), start.y + delta.y + speed * rdn2(randgen), angle};
            float x = rPose.x - MCLRobot.x;
            float y = rPose.y - MCLRobot.y;
            rPose.x = x * cosdTheta - y * sindTheta;
            rPose.y = x * sindTheta + y * cosdTheta;
            rPose += MCLRobot;

            //  create sample
            sample rSample = sample(
                rPose.x,
                rPose.y,
                angle,
                start.w);

            if (rSample.x > 70)
            {
                rSample.x = 70 - fmod(rSample.x, 70);
            }
            else if (rSample.x < -70)
            {
                rSample.x = -70 + fmod(-rSample.x, 70);
            }
            if (rSample.y > 70)
            {
                rSample.y = 70 - fmod(rSample.y, 70);
            }
            else if (rSample.y < -70)
            {
                rSample.y = -70 + fmod(-rSample.y, 70);
            }

            // put sample back
            samples[i] = rSample;

            sample tempRobot = samples[i];

            // distance sensor distance

            float weight = 1;
            if (GPSenable)
            {
                weight = gaussianWeight(tempRobot.distance({gpsx, gpsy}), 24, 2);

                samples[i].w *= weight;
            }
            weight = distanceSensorWeight(tempRobot, dc, backoffset);
            samples[i].w *= weight;

            weight = distanceSensorWeight(tempRobot, dl, leftoffset);
            samples[i].w *= weight;

            weight = distanceSensorWeight(tempRobot, dr, rightoffset);
            samples[i].w *= weight;

            weight = distanceSensorWeight(tempRobot, df, frontoffset);
            samples[i].w *= weight;
            // weight will adjust based off distance it should have been - distance it is
        }

        // std::cout<<"weight-"<<samples[1].w<< "   "<<1.0f/J<<std::endl;
        prevAngle = angle;
        prevRobot = Robot;
    }

    /**
     * @brief samples need to be normlaized
     *
     * @return float
     */
    float getESS()
    {
        normalizeSamples();
        float sum = 0;
        for (int i = 0; i < J; i++)
        {
            sample j = samples[i];
            sum += (j.w) * (j.w);
        }
        // std::cout << "sum:" << sum << std::endl;
        if (sum == 0)
        {
            return 0;
        }
        return 1 / sum;
    }

    float getStdDev()
    {
        normalizeSamples();
        float sumx = 0;
        float sumy = 0;
        for (int i = 0; i < J; i++)
        {
            sample j = samples[i];
            sumx += fabs(Robot.x - j.x);
            sumy += fabs(Robot.y - j.y);
        }
        float avgx = sumx / J;
        float avgy = sumy / J;
        return sqrt(avgx * avgx + avgy * avgy);
    }

    float meanWeight()
    {
        float sum = 0;
        for (int i = 0; i < J; i++)
        {
            sum += samples[i].w;
        }
        return sum / J;
    }
    /**
     * @brief move center
     *
     * @param c
     */
    void moveCenter(point c)
    {
        point center = getPose();
        for (int i = 0; i < J; i++)
        {
            point pt = samples[i];
            point delta = {pt.x - center.x, pt.y - center.y};
            pose particle = {c + delta, samples[i].theta};
            samples[i] = sample(particle, samples[i].w);
        }
    }
    /**
     * @brief resample MCL using stochastic universal resampling
     *
     */
    void resample()
    {
        // stochastic universal resampling
        std::vector<sample> temp(J);

        float start = (float)(((float)randgen() / (float)randgen.max()) / (float)J);

        float sum = samples[0].w;
        float j = start;

        // k is pointer
        float k = 0;

        // std::cout<<"start "<<start<<std::endl;
        //  repeat J times to fill entire space

        for (int i = 0; i < J; i++)
        {
            j = start + (float)((float)(i) / J);

            // while the weight we want is less than current sum, incremenet sum and counter
            while (j > sum)
            {
                k++;
                if (k > J - 1)
                    break;
                sum += samples[k].w;
            }
            // once we are less than sum, add to new sample
            if (k > J - 1)
            {
                j = 0;
                k = 0;
            }

            temp[i] = samples[k];
            temp[i].w = 1.0f / J;
        }
        samples = temp;
    }
    /**
     * @brief normalize MCL samples so sum of w is 1
     *
     */
    void normalizeSamples()
    {
        // std::cout << "Normalizing" << std::endl;
        float sum = 0;
        for (int i = 0; i < J; i++)
        {
            sum += samples[i].w;
        }
        // evertyhing is ok
        if (sum == 1)
        {
            // std::cout << "already 1: " << sum << std::endl;
            return;
        }
        // normalize everything
        for (int i = 0; i < J; i++)
        {
            samples[i].w /= sum;
        }
        // std::cout << "Normalized weights sum (should be1): " << sum << std::endl;
        return;
    }
    /**
     * @brief get current robot pose from MCL
     *
     * @return pose
     */
    pose getPose()
    {

        normalizeSamples();
        pose output{0, 0, 0};
        for (sample j : samples)
        {

            output.x += j.x * j.w;
            output.y += j.y * j.w;
            output.theta += j.theta * j.w;
        }
        return output;
    }
    void setParticleCount(int count)
    {
        if (count == J)
            return;

        if (count < J)
        {
            samples.erase(samples.begin() + count, samples.end());
        }
        else
        {
            pose start = Robot;
            auto rdn = std::normal_distribution<float>(0, 2);
            for (int i = J; i < count; i++)
            {
                float rx = start.x + rdn(randgen);
                float ry = start.y + rdn(randgen);
                float rt = start.theta;
                samples.push_back(sample(pose(rx, ry, rt), 1.0f / count));
            }
        }
        J = count;
        normalizeSamples();
    }
    /**
     * @brief main MCL thread
     *
     */
    void thread()
    {
        // wait(1, sec);
        Robot = pose(-60, -36, 0);
        // gyro1.setRotation(90, degrees);
        prevAngle = 90;
        // init(pose(robot, gyro1.rotation()), 1000);
        while (true)
        {
            // double start = Brain.timer(timeUnits::msec);
            update();
            if (meanWeight() * J < 1 * pow(10, -20))
            {
                std::cout << "kidnapped: " << MonteCarlo::meanWeight() << std::endl;
                // MonteCarlo::relocalize()
                spread *= 2;
            }
            else
            {
                spread = 1;
            }
            normalizeSamples();
            // if (getESS() < (J) / 2)
            // {
            //     // std::cout << "resampled" << std::endl;
            //     resample();
            // }
            resample();
            pose output = getPose();

            Robot.x = output.x;
            Robot.y = output.y;
            // wait(10, msec);
        }
    }

    //  RENDERING

    /**
     * @brief render distance sensors
     *
     */
    void renderDistanceSensors(SDL_Renderer *renderer)
    {
        std::normal_distribution<float> rdn{1, 0.025};
        int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
        int c = (1000 - boxSize) / 2;

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        // SDL_RenderDrawRect(renderer, new SDL_Rect{0, 0, 900, 900});
        float d = distanceReal(leftoffset, Robot) * rdn(randgen);

        if (d > 0)
        {
            pose offset = leftoffset;
            float sensorTheta = Robot.theta + offset.theta;
            float thetaRad = (90 - sensorTheta) * M_PI / 180;

            // precompute sin and cos
            float sinTheta = sin(thetaRad);
            float cosTheta = cos(thetaRad);

            float thetaRad2 = (-Robot.theta) * M_PI / 180;
            float sinTheta1 = sin(thetaRad2);
            float cosTheta1 = cos(thetaRad2);

            point offsetdelta = point(
                offset.x * cosTheta1 - offset.y * sinTheta1,
                offset.x * sinTheta1 + offset.y * cosTheta1);
            // offset.print();
            // offsetdelta.print();
            // in px
            float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
            float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            //*SCALE_FACTOR
            float endX = sensorX + d * SCALE_FACTOR * cosTheta;
            float endY = sensorY - d * SCALE_FACTOR * sinTheta;

            SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX - 5, (int)sensorY - 5, 10, 10});
            SDL_RenderDrawLine(renderer,
                               sensorX,
                               sensorY,
                               endX,
                               endY);
        }

         d = distanceReal(backoffset, Robot) * rdn(randgen);

        if (d > 0)
        {
            pose offset = backoffset;
            float sensorTheta = Robot.theta + offset.theta;
            float thetaRad = (90 - sensorTheta) * M_PI / 180;

            // precompute sin and cos
            float sinTheta = sin(thetaRad);
            float cosTheta = cos(thetaRad);

            float thetaRad2 = (-Robot.theta) * M_PI / 180;
            float sinTheta1 = sin(thetaRad2);
            float cosTheta1 = cos(thetaRad2);

            point offsetdelta = point(
                offset.x * cosTheta1 - offset.y * sinTheta1,
                offset.x * sinTheta1 + offset.y * cosTheta1);
            // offset.print();
            // offsetdelta.print();
            // in px
            float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
            float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            //*SCALE_FACTOR
            float endX = sensorX + d * SCALE_FACTOR * cosTheta;
            float endY = sensorY - d * SCALE_FACTOR * sinTheta;

            SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX - 5, (int)sensorY - 5, 10, 10});
            SDL_RenderDrawLine(renderer,
                               sensorX,
                               sensorY,
                               endX,
                               endY);
        }
        d = distanceReal(rightoffset, Robot) * rdn(randgen);

        if (d > 0)
        {
            pose offset = rightoffset;
            float sensorTheta = Robot.theta + offset.theta;
            float thetaRad = (90 - sensorTheta) * M_PI / 180;

            // precompute sin and cos
            float sinTheta = sin(thetaRad);
            float cosTheta = cos(thetaRad);

            float thetaRad2 = (-Robot.theta) * M_PI / 180;
            float sinTheta1 = sin(thetaRad2);
            float cosTheta1 = cos(thetaRad2);

            point offsetdelta = point(
                offset.x * cosTheta1 - offset.y * sinTheta1,
                offset.x * sinTheta1 + offset.y * cosTheta1);
            // offset.print();
            // offsetdelta.print();
            // in px
            float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
            float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            //*SCALE_FACTOR
            float endX = sensorX + d * SCALE_FACTOR * cosTheta;
            float endY = sensorY - d * SCALE_FACTOR * sinTheta;

            SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX - 5, (int)sensorY - 5, 10, 10});
            SDL_RenderDrawLine(renderer,
                               sensorX,
                               sensorY,
                               endX,
                               endY);
        }


         d = distanceReal(frontoffset, Robot) * rdn(randgen);

        if (d > 0)
        {
            pose offset = frontoffset;
            float sensorTheta = Robot.theta + offset.theta;
            float thetaRad = (90 - sensorTheta) * M_PI / 180;

            // precompute sin and cos
            float sinTheta = sin(thetaRad);
            float cosTheta = cos(thetaRad);

            float thetaRad2 = (-Robot.theta) * M_PI / 180;
            float sinTheta1 = sin(thetaRad2);
            float cosTheta1 = cos(thetaRad2);

            point offsetdelta = point(
                offset.x * cosTheta1 - offset.y * sinTheta1,
                offset.x * sinTheta1 + offset.y * cosTheta1);
            // offset.print();
            // offsetdelta.print();
            // in px
            float sensorX = (c + boxSize / 2) + (Robot.x + offsetdelta.x) * SCALE_FACTOR;
            float sensorY = (c + boxSize / 2) - (Robot.y + offsetdelta.y) * SCALE_FACTOR;

            SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
            //*SCALE_FACTOR
            float endX = sensorX + d * SCALE_FACTOR * cosTheta;
            float endY = sensorY - d * SCALE_FACTOR * sinTheta;

            SDL_RenderDrawRect(renderer, new SDL_Rect{(int)sensorX - 5, (int)sensorY - 5, 10, 10});
            SDL_RenderDrawLine(renderer,
                               sensorX,
                               sensorY,
                               endX,
                               endY);
        }
        // std::cout << std::endl;
    }

    void renderParticles(SDL_Renderer *renderer)
    {
        int boxSize = static_cast<int>(140.75 * SCALE_FACTOR);
        int c = (1000 - boxSize) / 2;
        SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
        auto maxElement = std::max_element(samples.begin(), samples.end(),
                                           [](const sample &a, const sample &b)
                                           {
                                               return a.w < b.w;
                                           });
        float maxW = maxElement->w;
        // std::cout << "particles" << std::endl;
        for (int i = 0; i < J; i++)
        {
            sample j = samples[i];
            // j.print();
            float x = (c + boxSize / 2) + j.x * SCALE_FACTOR;
            float y = (c + boxSize / 2) - j.y * SCALE_FACTOR;
            int a = remap(j.w, 0, maxW, 0, 255);
            SDL_SetRenderDrawColor(renderer, 255 - a, 255, 0, 100);
            SDL_FRect rect = {x - 2, y - 2, 4, 4}; // Adjust the size of the rectangle as needed
            SDL_RenderFillRectF(renderer, &rect);
        }
    }
};