#include "stdio.h"

#include <math.h>
#include <iostream>
#include <algorithm>
#include <monteCarlo.h>
/*
 ██████╗   ██████╗    ██████╗   ███╗   ███╗  ███████╗  ████████╗  ███████╗  ██████╗   ██╗   ██╗
██╔═══██╗  ██╔══██╗  ██╔═══██╗  ████╗ ████║  ██╔════╝  ╚══██╔══╝  ██╔════╝  ██╔══██╗  ╚██╗ ██╔╝
██║   ██║  ██║  ██║  ██║   ██║  ██╔████╔██║  █████╗       ██║     █████╗    ██████╔╝   ╚████╔╝
██║   ██║  ██║  ██║  ██║   ██║  ██║╚██╔╝██║  ██╔══╝       ██║     ██╔══╝    ██╔══██╗    ╚██╔╝
╚██████╔╝  ██████╔╝  ╚██████╔╝  ██║ ╚═╝ ██║  ███████╗     ██║     ███████╗  ██║  ██║     ██║
 ╚═════╝   ╚═════╝    ╚═════╝   ╚═╝     ╚═╝  ╚══════╝     ╚═╝     ╚══════╝  ╚═╝  ╚═╝     ╚═╝

*/

pose Robot = pose(0, 0,0);

#include <string>
#include <vector>

void point::print()
{
	std::cout << x << "," << y << std::endl;
}

float point::distance(point p)
{
	return sqrt(pow(p.x - this->x, 2) + pow(p.y - this->y, 2));
}
float point::angleTo(point p1)
{
	return atan2(p1.x - this->x, p1.y - this->y) * 180 / M_PI;
}

bool point::equals(point p1)
{
	if ((std::round(this->x) == std::round(p1.x)) and (std::round(this->y) == std::round(p1.y)))
	{
		return true;
	}
	return false;
}

point point::operator+(point p1)
{
	return point(this->x + p1.x, this->y + p1.y);
}

point point::operator+=(point p1)
{
	this->x += p1.x;
	this->y += p1.y;
	return *this;
}

/**
 * @brief odometry
 */
int odometeryThread()
{
	Odometry::init();
	while (true)
	{
		pose delta = Odometry::odomStep();
		Robot += delta;
		//this_thread::sleep_for(10);
	}
	return 1;
}
double Odometry::prevLE = 0;
double Odometry::prevRE = 0;
double Odometry::prevBE = 0;
double Odometry::prevAngle = 0;
/**
 * @brief init odom
 * 
 */
void Odometry::init()
{
	// RF.resetPosition();
	// LF.resetPosition();
	// gyro1.resetHeading();
	// waitUntil(!gyro1.isCalibrating());
	// Robot = point(0, 0);
	// RF.resetPosition();
	// LF.resetPosition();
	// RF.setPosition(0, degrees);
	// LF.setPosition(0, degrees);
	
}
/**
 * @brief automatic odomstep
 *
 * @return delta pose
 */
pose Odometry::odomStep()
{
	// angle
	// float angle = gyro1.rotation();
	// float dAngle = angle - prevAngle;
	// prevAngle = angle;

	// // encoders
	// float lEncoder = LF.position(degrees) * 3 / 4;
	// float rEncoder = RF.position(degrees) * 3 / 4;
	// float bEncoder = rotationB.position(degrees);

	// float dLE = lEncoder - prevLE;
	// float dRE = rEncoder - prevRE;
	// float dBE = bEncoder - prevBE;

	// prevBE = bEncoder;
	// prevLE = lEncoder;
	// prevRE = rEncoder;

	// trigger odomstep
	return pose(); //odomStep(dLE, dRE, dBE, angle, dAngle);
}
/**
 * @brief odom step with specicified variables
 *
 * @param dLE delta left encoder (degrees)
 * @param dRE delta right encoder (degrees)
 * @param dBE delta back encoder (degrees)
 * @param angle angle (degrees)
 * @param dAngle delta angle (degrees)
 * @return pose
 */
pose Odometry::odomStep(float dLE, float dRE, float dBE, float angle, float dAngle)
{
	//  convert encoder distance into distance traveled
	float distL = ((dLE)*M_PI / 180) * lRad;
	float distR = ((dRE)*M_PI / 180) * rRad;
	float distB = ((dBE)*M_PI / 180) * bRad;

	float Heading = ((angle * M_PI / 180));
	Heading = fmod(Heading, 2 * M_PI);
	float deltaHeading = dAngle * (M_PI / 180); // calculate change in heading

	float deltaX = 0;
	float deltaY = 0;

	if (deltaHeading == 0)
	{
		deltaX = distB;
		deltaY = (distL + distR) / 2;
	}
	else
	{
		deltaX = 2 * sin(deltaHeading / 2) * ((distB / deltaHeading) + distFromCenterB);
		deltaY = 2 * sin(deltaHeading / 2) * ((distR / deltaHeading) - distFromCenterL);
	}

	float averageHeading = Heading - (deltaHeading / 2);

	pose output;
	output.x = (deltaY * sin(averageHeading)) + (deltaX * cos(averageHeading));
	output.y += (deltaY * cos(averageHeading)) - (deltaX * sin(averageHeading));
	output.theta = angle;
	return output;
}
