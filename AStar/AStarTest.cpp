#ifndef NDEBUG
#define NDEBUG
#endif
#include <iostream>
#include <fstream>
#include <queue>
#include "AStarNode.h"
#include <ctime>

#define MAX_STEPS 30000

int main(int argc, char** argv)
{
    clock_t start;
    start = std::clock();
//    omp_set_num_threads(4);
    double distToGoal;
    //Set up the priority quee
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodePtrCompare> openQueue;
    std::vector<AStarNode*> expandedNodes;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> initState;
    initState << 0,0,0,0,0,0;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState;
    goalState << -0.2, -0.0525, 0.1, 0.0245, 0.4115, 0.8511;
    openQueue.push(new AStarNode(initState,goalState));

    AStarNode* currentNode; //Temporary node to hold whats in The queue

    bool solFound = 0;
    int count = 0;
    while(!openQueue.empty() && !solFound && count < MAX_STEPS)
    {
	count++;
	if(count % 500 == 0)
	    std::cout << "Step: " << count << std::endl;
	//Pop the highest priority node (lowest cost)
	currentNode = openQueue.top();
	openQueue.pop();
//	closedNodes.push_back(currentNode);

	//Check if the node is close enough
	if(currentNode->getNodeCostToGo() <= GOAL_EPSILON) {
	    std::cout << "Done!" << std::endl;
	    solFound = 1;
	}
	//
	expandedNodes = currentNode->expand();
	//Add to queue
	while(!expandedNodes.empty())
	{
	    openQueue.push(expandedNodes.back());
	    expandedNodes.pop_back();
	}
    }
    std::cout<<"Cost to Go: "<< currentNode->getNodeCostToGo()<<std::endl;
    //Last node
    std::cout << "Last node cost to come: " << currentNode->getNodeCostToCome() << std::endl;
    //Save the solution
    std::vector<AStarNode*> solution;
    AStarNode* solNode = currentNode;
    AStarNode* prevNode;
    while(solNode->getNodeParent() != NULL) {
	solution.push_back(solNode);
	prevNode = solNode;
	solNode = solNode->getNodeParent();
	std::cout<<"Cost To Go: "<<solNode->getNodeCostToGo()<<std::endl;
	if(solNode == prevNode) {
	    std::cout << "weird pointer behavior" << std::endl;
	    break;
	}
    }
    //Add the root node
    solution.push_back(solNode);
    std::cout<<"Solution has " << solution.size() << " nodes."<<std::endl;
    std::cout << "Outputting files" << std::endl;
    std::ofstream out_file("Solution.txt", std::ios::trunc);
    out_file <<"X Xd Y Yd Th Thd s Fn Ft"<<std::endl;
    Eigen::Matrix<double, STATE_SPACE_DIM, 1> tempState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM, 1> tempControl;
    while(!solution.empty() && out_file.is_open()) {
	currentNode = solution.back();
	solution.pop_back();
	tempState = currentNode->getNodeState();
	tempControl = currentNode->getNodeControl();
	out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
		 << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
	         << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0) << std::endl;
    }
    std::cout<<"Time: " << (std::clock() - start)/(double)CLOCKS_PER_SEC<<std::endl;
    return 0;
}
