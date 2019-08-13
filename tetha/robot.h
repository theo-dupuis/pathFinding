#pragma once
#include <vector>
#include "obstacle.h"

class Robot
{
private:
	static std::vector<Obstacle> obstacles;
	int x_;
	int y_;
	int radius_;
	float angle_;
public:
	Robot(int x, int y, int radius,float angle);
	Robot(int x, int y, int radius);
	
	int getX(void) {return x_;};
	int getY(void) {return y_;};
	int getRadius(void) {return radius_;};
	float getAngle(void) {return angle_;};
	
	static void addObstacle(Obstacle o) {obstacles.push_back(o);};
	void setPosition(int x, int y) {x_=x; y_ = y;};
	
	bool collide();
};