#define NDEBUG
#include <iostream>
#include <fstream>
#include <queue>
#include "SPARSE-RRTNode.h"
#include <ctime>
#include <string>

#define MAX_SAMPLES 60000

int main(int argc, char** argv)
{
    clock_t start;
    start = std::clock();
    std::string filename;
    if(argc > 1) {
	filename = argv[1];
    } else {
	filename = "Solution.txt";
    }
    double distToGoal;
    //Set up the priority quee
    int discardedSamples = 0;
    std::vector<SPARSE_RRTNode*> tree;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> initState;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> basicControlSample;
    initState << 0, 0, 0, 0, 0, 0;
    goalState << -0.3, 0.0, 0.0, -0.95, M_PI, 0.0;
    basicControlSample << 0, GRAV*MO, 0;
    tree.push_back(new SPARSE_RRTNode(initState));
    bool solFound = 0;
    int count = 0;

    Eigen::Matrix<double, STATE_SPACE_DIM,1> sampleState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> sampleControl;
    SPARSE_RRTNode* nearest;
    std::vector<SPARSE_RRTNode*> nearestVector;
    double minDist;
    
    while(!solFound && count < MAX_SAMPLES)
    {
	//sample a random state
	sampleState = RandomSample(goalState);
	//Find its nearest neighbor in the tree
	nearest = tree[0];
	minDist = dist(sampleState,nearest->getNodeState());
	for(int i = 1; i < tree.size(); i++) {
	    if((dist(sampleState,tree[i]->getNodeState())) < minDist) {
		minDist = dist(sampleState,tree[i]->getNodeState());
		nearest = tree[i];
	    }
	    if((dist(sampleState,tree[i]->getNodeState)) < DELTA_NEAR) {
		nearestVector.pushback(tree[i]);
	    }
	}
	if(!nearestVector.empty()) {
	    double minTime = nearestVector[0].getNodeTime();
	    nearest = nearestVector[0];
	    for(int i = 1; i < nearestVector.size(); i++) {
		if(nearestVector[i].getNodeTime() < minTime) {
		    minTime = nearestVector[i].getNodeTime();
		    nearest = nearestVector[i];
		}
	    }
	}
	Eigen::Matrix<double,CONTROL_SPACE_DIM,1> bestControl;
	Eigen::Matrix<double,STATE_SPACE_DIM,1> bestState;
	Eigen::Matrix<double,STATE_SPACE_DIM,1> tempState;
	sampleControl = RandomControl(nearest->getNodeControl());
	double bestDist;
	bestState = spawn(nearest->getNodeState(),sampleControl);
	bestDist = dist(bestState, sampleState);
	//Randomly sapmle control space and pick the "best"
	for(int i = 0; i < 5; i++) {
	    sampleControl = RandomControl(nearest->getNodeControl());
	    tempState = spawn(nearest->getNodeState(),sampleControl);
	    if(dist(tempState,sampleState) < bestDist) {
		bestDist = dist(tempState,sampleState);
		bestControl = sampleControl;
		bestState = tempState;
	    }
	}
	tree.push_back(new SPARSE_RRTNode(bestState, bestControl, nearest, nearest->getNodeTime() + TIME_STEP));

	count++;

	if(dist(goalState,tree.back()->getNodeState()) <= GOAL_EPSILON) {
	    solFound = 1;
	}
    }
    SPARSE_RRTNode* solNode;
    if(!solFound) {
	solNode = tree[0];
	minDist = dist(solNode->getNodeState(), goalState);
	double tempDist;
	for(int i = 1; i < tree.size(); i++) {
	    tempDist = dist(tree[i]->getNodeState(), goalState);
	    if(tempDist < minDist) {
		minDist = tempDist;
		solNode = tree[i];
	    }
	}
    } else {
	solNode = tree.back();
    }
    // Output solution distance for batch
    std::cout<<minDist<<std::endl;
    //Save the solution
    std::vector<SPARSE_RRTNode*> solution;
    SPARSE_RRTNode* prevNode;
    while(solNode->getNodeParent() != NULL) {
	solution.push_back(solNode);
	prevNode = solNode;
	solNode = solNode->getNodeParent();
    }
    solution.push_back(solNode);

    std::ofstream out_file(filename, std::ios::trunc);
    out_file <<"X Xd Y Yd Th Thd s Fn Ft"<<std::endl;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> tempState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> tempControl;
    SPARSE_RRTNode* currentNode;
    while(!solution.empty() && out_file.is_open()) {
	currentNode = solution.back();
	solution.pop_back();
	tempState = currentNode->getNodeState();
	tempControl = currentNode->getNodeControl();
	out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
		 << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
	         << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0) << std::endl;
    }
    return 0;
}
