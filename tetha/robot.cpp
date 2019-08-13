#include "robot.h"

std::vector<Obstacle> Robot::obstacles;
Robot::Robot(int x, int y, int radius,float angle)
{
	x_ = x;
	y_ = y;
	radius_ = radius;
	angle_ = angle;
}

Robot::Robot(int x, int y, int radius)
{
	x_ = x;
	y_ = y;
	radius_ = radius;
}

bool Robot::collide()
{
	if (x_ <radius_ || x_+radius_ > 3000 || y_<radius_ || y_+radius_ > 2000) 
		return true;
	int nearestX;
	int nearestY;
	
	for(Obstacle obstacle : obstacles)
	{	
		if (x_ > obstacle.getXMin())
		{
			if (x_ < obstacle.getXMax())
				nearestX = x_;
			else
				if (x_ - radius_>obstacle.getXMax())
					continue;
				else
					nearestX = obstacle.getXMax();
		}
		else
			if (x_ + radius_ < obstacle.getXMin())
				continue;
			else
				nearestX = obstacle.getXMin();
		
		if (y_ > obstacle.getYMin())
		{

			if (y_ < obstacle.getYMax())
				nearestY = y_;
			else
				if (y_ - radius_ > obstacle.getYMax())
					continue;
				else
					nearestY = obstacle.getYMax();
		}
		else
			if (y_ + radius_ < obstacle.getYMin())
				continue;
			else
				nearestY = obstacle.getYMin();
		
		int deltaX = x_;
		deltaX -= nearestX;
		int deltaY = y_;
		deltaY -= nearestY;
		if ((deltaX *deltaX + deltaY*deltaY) < (radius_ * radius_))
			return true;
	}
	return false;
}