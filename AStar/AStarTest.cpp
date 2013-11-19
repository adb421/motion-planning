#ifndef NDEBUG
#define NDEBUG
#endif
#include <iostream>
#include <fstream>
#include <queue>
#include "AStarNode.h"
#include <ctime>

#define MAX_STEPS 20000

int main(int argc, char** argv)
{
    clock_t start;
    start = std::clock();
//    omp_set_num_threads(4);
    double distToGoal;
    //Set up the priority quee
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodePtrCompare> openQueue;
    std::vector<AStarNode*> closedNodes;
    std::vector<AStarNode*> expandedNodes;
    std::array<double, STATE_SPACE_DIM> initState = {0,0,0,0,0,0};
//    std::array<double, STATE_SPACE_DIM> goalState = {-0.2, 0.0, -0.1, 0.0, 0.0, 0.0};
    std::array<double, STATE_SPACE_DIM> goalState = {-0.3, 0.0, 0.0, -3.0, M_PI, 0.0};
//    std::array<double, STATE_SPACE_DIM> goalState = {0,0,0,0,0,0};
//    std::array<double, STATE_SPACE_DIM> initState = {-0.5, 0.0, 0.0, -10.0, M_PI, 0.0};
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
	closedNodes.push_back(currentNode);

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
//	std::cout << solNode->getNodeParent() << std::endl;
	solution.push_back(solNode);
	prevNode = solNode;
	solNode = solNode->getNodeParent();
	std::cout<<"Cost To Go: "<<solNode->getNodeCostToGo()<<std::endl;
//	std::cout << solNode << std::endl;
//	std::cout << solNode->getNodeParent() << std::endl;
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
    std::array<double, STATE_SPACE_DIM> tempState;
    std::array<double, CONTROL_SPACE_DIM> tempControl;
    while(!solution.empty() && out_file.is_open()) {
	currentNode = solution.back();
	solution.pop_back();
	tempState = currentNode->getNodeState();
	tempControl = currentNode->getNodeControl();
	out_file << tempState[0] << " " << tempState[1] << " " << tempState[2] << " " \
		 << tempState[3] << " " << tempState[4] << " " << tempState[5] << " " \
	         << tempControl[0] << " " << tempControl[1] << " " << tempControl[2] << std::endl;
    }
    std::cout << "Deleting open nodes" << std::endl;
    //Delete!!!! openQueue and closedNodes
    while(!openQueue.empty()) {
	delete openQueue.top();
	openQueue.pop();
    }
    std::cout << "Deleting closed nodes" << std::endl;
    while(!closedNodes.empty()) {
	delete closedNodes.back();
	closedNodes.pop_back();
    }
    std::cout<<"Time: " << (std::clock() - start)/(double)CLOCKS_PER_SEC<<std::endl;
    return 0;
}
