#include <math.h>
#include <chrono>
#include <iostream>
#include "pathFinding.h"
#include "robot.h"
#include "obstacle.h"
int main()
{
	std::chrono::time_point<std::chrono::system_clock> start, end;
	//Robot robot(2765,445,146, 3.14159265,4, (float)0.017453293005625408);
	Robot robot(2765, 445, 146, 3.14159265);
	Robot::addObstacle(Obstacle(1485,1525,1370,2000));
	Robot::addObstacle(Obstacle(500,2510,0,20));
	Robot::addObstacle(Obstacle(450,2550,1570,2000));
	
	PathFinding pathFinding(robot);
	int elapsed_seconds = 0;
	start = std::chrono::system_clock::now();
	for (int i = 0; i < 10; i++)
	{
		//pathFinding.find(1350, 855);

		start = std::chrono::system_clock::now();
		pathFinding.find(180, 1820);
		end = std::chrono::system_clock::now();
		elapsed_seconds += std::chrono::duration_cast<std::chrono::milliseconds>
			(end - start).count();
		pathFinding.clearContainers();
	}

	end = std::chrono::system_clock::now();
	std::cout << "elapsed time: " << elapsed_seconds << "ms\n";
	system("pause");
	return 0;
}