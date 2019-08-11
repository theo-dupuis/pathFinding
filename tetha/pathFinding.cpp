#include "pathFinding.h"
#include <stdio.h>
#include <math.h>

PathFinding::PathFinding(Robot &robot)
{
	robot_ = &robot;
}

void PathFinding::find(int x, int y)
{
	Robot robot(x,y,robot_->getRadius());
	if (robot.collide()) return;
	explored_.clear();
	grid_.clear();
	grid_.reserve(400);
	nodes_.clear();
	xGoal_ = x;
	yGoal_ = y;

	nodes_.push_back(Node(*this, 0, robot_->getX(), robot_->getY(), 0, 0));
	grid_.push_back(&(nodes_.front()));
	
	bool finished = false;
	Node* currentNode;
	do
	{
		currentNode = grid_[0];
		grid_.erase(grid_.begin());
		finished=currentNode->explore();
	}while(!finished);
	
	printf("%d noeuds explores\n", explored_.size());

	if (checkTrajectoire(currentNode->getX(), currentNode->getY(), xGoal_, yGoal_))
		currentNode = currentNode->getParent();
	printf("aller à (%d,%d)\n",xGoal_, yGoal_);

	while(currentNode->getParent() !=0)
	{
		printf("aller à (%d,%d)\n",currentNode->getX(), currentNode->getY());
		currentNode = currentNode->getParent();
	}
}

bool PathFinding::checkTrajectoire(int x1, int y1, int x2, int y2)
{
	int deltaX = x2-x1;
    int deltaY = y2-y1;
	float norm = STEP/sqrt(deltaX*deltaX + deltaY*deltaY);
    deltaX *=  norm;
    deltaY *=  norm;
	Robot robot(x1,y1,robot_->getRadius());
	if (robot.collide()) return false;
	
	while (robot.getX()!= x2 && robot.getY()!=y2)
	{
		if(((x2-robot.getX())*(x2-robot.getX())+(y2-robot.getY())*(y2-robot.getY()))< STEP*STEP)
		{
			robot.setPosition(x2,y2);
		}
		else
		{
			robot.setPosition(robot.getX()+deltaX,robot.getY()+deltaY);
		}
		if (robot.collide()) return false;
	}
	return true;
}

float PathFinding::eval(int x1, int y1, int x2, int y2)
{
	int deltaX = x2-x1;
	int deltaY = y2-y1;
	return (deltaX*deltaX+deltaY*deltaY);
}

PathFinding::Node::Node(PathFinding &pathFinding, Node* parent, int x, int y, float evaluation, float time)
{
	pathFinding_ = &pathFinding;
	parent_ = parent;
	x_ = x;
	y_ = y;
	evaluation_ = evaluation;
	time_ = time;
}

PathFinding::Node::Node(Node &other)
{
	pathFinding_ = other.pathFinding_;
	parent_ = other.parent_;
	x_ = other.x_;
	y_ = other.y_;
	evaluation_ = other.evaluation_;
	time_ = other.time_;
}

void PathFinding::Node::checkNode(int x, int y)
{
	Robot robot(x,y,pathFinding_->robot_->getRadius());
	if (robot.collide()) return;
	Node* nextParent;
	if (parent_!= 0 && pathFinding_->checkTrajectoire(parent_->x_,parent_->y_,x,y))
		nextParent = parent_;
	else
		nextParent = this;
	float nextTime = nextParent->time_ + pathFinding_->eval(nextParent->x_, nextParent->y_, x,y);
	float nextEvaluation = nextTime + pathFinding_->eval(x,y, pathFinding_->xGoal_, pathFinding_->yGoal_);
	
	int min=0;
	int max= pathFinding_->grid_.size();
	int i=0;
	while (min != max)
	{
	  i= (min+max)/2;
	  if (nextEvaluation <= (pathFinding_->grid_[i]->evaluation_))
		max = i;
	  else
	  {
		if (i==min)
		  break;
		min = i;
	  }
	}
	pathFinding_->nodes_.push_back(Node(*pathFinding_, nextParent, x, y, nextEvaluation, nextTime));
	pathFinding_->grid_.insert(pathFinding_->grid_.begin()+max,&(pathFinding_->nodes_.back()));
}

bool PathFinding::Node::explore()
{
	int code = ((x_/5)<<8)+y_/5;
	if (pathFinding_->explored_.find(code)!= pathFinding_->explored_.end()) return false;
	int nextX;
	int nextY;
	
	pathFinding_->explored_.insert(code);
	
	int goalDiffX = pathFinding_->xGoal_- x_;
	int goalDiffY = pathFinding_->yGoal_ - y_;
	if ((goalDiffX*goalDiffX + goalDiffY*goalDiffY) < (pathFinding_->STEP * pathFinding_->STEP))
	{
		return true;
	}
	for (int i = -1;i<2;i++)
		for (int j= -1; j<2;j++)
		{
			if (!(i | j))
				continue;
			nextX = x_+i*GRIDSIZE;
			nextY = y_+j*GRIDSIZE;
			if(pathFinding_->explored_.find(((nextX/5)<<8)+nextY/5)!= pathFinding_->explored_.end())
				continue;
			checkNode(nextX,nextY);
		}
	return false;
}