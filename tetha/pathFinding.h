#pragma once
#include <vector>
#include <list>
#include <set>
#include "robot.h"

class PathFinding
{
private:
	class Node
	{
	private:
		PathFinding* pathFinding_;
		Node* parent_;
		int x_;
		int y_;
		float angle_;
		float evaluation_;
		float time_;
	public:
		Node(PathFinding &pathFinding, Node* parent, int x, int y, float angle, float evaluation, float time);
		Node(Node &other);
		int getX() {return x_;};
		int getY() {return y_;};
		Node* getParent() {return parent_;};
		
		void checkNode(int x, int y);
		bool explore();
	};
	
	Robot* robot_;
	const int STEP = 100;
	static const int GRIDSIZE = 50;
	int xGoal_;
	int yGoal_;
	std::set<int> explored_;
	std::vector<Node*> grid_;
	std::list<Node> nodes_;
public:
	PathFinding(Robot &robot);
	
	void find(int x, int y);
	bool checkTrajectoire(int x1, int y1, int x2, int y2);
	float eval(int x1, int y1, float angle, int x2, int y2);
};