#pragma once

class Obstacle
{
private:
	int xMin_;
	int xMax_;
	int yMin_;
	int yMax_;
public:
	Obstacle(int xMin, int xMax, int yMin, int yMax);
	
	int getXMin(void) {return xMin_;};
	int getXMax(void) {return xMax_;};
	int getYMin(void) {return yMin_;};
	int getYMax(void) {return yMax_;};
};