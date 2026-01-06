#pragma once

class point
{
public:
  float x;
  float y;
  point(float x, float y) : x(x), y(y) {}
  point() : x(0), y(0) {}
  float distance(point p);
  point operator+(point p1);
  point operator+=(point p1);
  float robotDistance();
  bool equals(point p1);
  float angleTo(point p1);
  void print();
};

#include "monteCarlo.h"
class pose;
extern pose Robot;
int odometeryThread();

class Odometry
{
private:
  static constexpr double distFromCenterL = 6;
  static constexpr double distFromCenterB = 1;
  static constexpr double lRad = 1.625;
  static constexpr double rRad = 1.625; // radius of tracking wheel
  static constexpr double bRad = 1.375; // radius of tracking wheel

  static double prevLE; // create previous encoder value left
  static double prevRE; // create previous encoder value right
  static double prevBE; // create previous encoder value right
  static double prevAngle;


  

public:
  static pose odomStep();
  static void init();
  static pose odomStep(float dLE, float dRE, float dBE, float angle, float dAngle);
};