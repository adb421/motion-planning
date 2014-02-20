#ifndef NDEBUG
#define NDEBUG
#endif
#include <iostream>
#include <fstream>
#include <queue>
#include "AStarNode.h"
#include <ctime>

#define MAX_STEPS 50000
int main(int argc, char** argv)
{
    clock_t start;
    start = std::clock();
//    omp_set_num_threads(4);
    double distToGoal;
    double bestEucDist;
    //Set up the priority queue
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNodePtrCompare> openQueue;
    std::vector<AStarNode*> expandedNodes;
    map_t grid;
    grid.clear();
    StateVector_t initState;
    initState << 0.0, 0.0;
    StateVector_t goalState;
    goalState << 0.2, 0.5;
    AStarNode* currentNode; //Temporary node to hold whats in The queue
    AStarNode* bestNode;
    currentNode = new AStarNode(initState, goalState);
    std::array<int, STATE_SPACE_DIM> tmpArray = snapToGrid(currentNode);
    bool insCheck =grid.insert(std::make_pair(tmpArray,currentNode)).second;

    bestNode = currentNode;
    bestEucDist = euclideanDistance(initState,goalState);
    openQueue.push(currentNode);
    int badCount = 0;
    bool solFound = 0;
    int count = 0;
    while(!openQueue.empty() && !solFound && count < MAX_STEPS)
    {
	count++;
	if(count % 50 == 0)
	    std::cout << "Step: " << count << std::endl;
	//Pop the highest priority node (lowest cost)
	currentNode = openQueue.top();
	openQueue.pop();
	while(!currentNode->good){ // if it was replaced
	    if(openQueue.empty()) {
		std::cout<<"Open node list is empty, nothing to search!"<<std::endl;
		break;
	    }
	    badCount++;
	    currentNode = openQueue.top();
	    openQueue.pop();
	}
	if(euclideanDistance(currentNode->getNodeState(),goalState) < bestEucDist) {
	    bestEucDist = euclideanDistance(currentNode->getNodeState(),goalState);
	    bestNode = currentNode;
	}
//	closedNodes.push_back(currentNode);
	//Check if the node is close enough
	if(bestEucDist <= GOAL_EPSILON) {
	    std::cout << "Done!" << std::endl;
	    solFound = 1;
	}
	//
	expandedNodes = currentNode->expand(grid);
	//Add to queue
	while(!expandedNodes.empty())
	{
	    openQueue.push(expandedNodes.back());
	    expandedNodes.pop_back();
	}
	if(openQueue.empty()) {
	    std::cout<<"Empty queue!"<<std::endl;
	}
    }
    if(euclideanDistance(currentNode->getNodeState(),goalState) < bestEucDist) {
	bestEucDist = euclideanDistance(currentNode->getNodeState(),goalState);
	bestNode = currentNode;
    }
    currentNode = bestNode; //Get best node so far
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
    out_file <<"X Xd Fn"<<std::endl;
    StateVector_t tempState;
    ControlVector_t tempControl;
    while(!solution.empty() && out_file.is_open()) {
	currentNode = solution.back();
	solution.pop_back();
	tempState = currentNode->getNodeState();
	tempControl = currentNode->getNodeControl();
	for(int k = 0; k < STATE_SPACE_DIM; k++) {
	    out_file << tempState(k,0) << " ";
	}
	for(int k = 0; k < CONTROL_SPACE_DIM; k++) {
	    out_file << tempControl(k,0) << " ";
	}
	out_file << std::endl;
    }
    std::cout<<"Time: " << (std::clock() - start)/(double)CLOCKS_PER_SEC<<std::endl;
    std::cout<<"Grid size is: "<<grid.size()<<std::endl;
    std::cout<<"Bad Count: "<<badCount<<std::endl;
//    printBadCount();
    return 0;
}
