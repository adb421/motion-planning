#define NDEBUG
#include <iostream>
#include <fstream>
#include <queue>
#include "SPARSE-RRTNode.h"
#include <ctime>
#include <string>

#define MAX_SAMPLES 100

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

    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> goalControl;
    initState << 0, 0, 0, 0, 0, 0;
    goalState << -0.2, -0.0525, 0.1, 0.0245, 0.4115, 0.8511; //Should get us from dyn grasp to roll
    std::array<int,CONTROL_SPACE_DIM> controlCare = {0,0,1,1};
    goalControl << 0.0, 0.0, 0.0, 0.0;
    //    goalControl << -LO, 0.0, 0.0; //Only care about control input 1 anyway
//    basicControlSample << 0, GRAV*MO, 0;
    basicControlSample << 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN;
    tree.push_back(new SPARSE_RRTNode(initState));
    bool solFound = 0;
    int count = 0;

    Eigen::Matrix<double, STATE_SPACE_DIM,1> sampleState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> sampleControl;
    SPARSE_RRTNode* nearest;
    std::vector<SPARSE_RRTNode*> nearestVector;
    double minDist;
    SPARSE_RRTNode* solNode;
    double solDist;
    solNode = tree.back();
    solDist = dist(solNode->getNodeState(),goalState);
    while(!solFound && count < MAX_SAMPLES)
    {
	int goodSample = 0;
	Eigen::Matrix<double,CONTROL_SPACE_DIM,1> bestControl;
	Eigen::Matrix<double,STATE_SPACE_DIM,1> bestState;
	while(!goodSample) {
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
		// if((dist(sampleState,tree[i]->getNodeState())) < DELTA_NEAR) {
		//     nearestVector.push_back(tree[i]);
		// }
	    }
	    // if(!nearestVector.empty()) {
	    // 	double minTime = nearestVector[0]->getNodeTime();
	    // 	nearest = nearestVector[0];
	    // 	for(int i = 1; i < nearestVector.size(); i++) {
	    // 	    if(nearestVector[i]->getNodeTime() < minTime) {
	    // 		minTime = nearestVector[i]->getNodeTime();
	    // 		nearest = nearestVector[i];
	    // 	    }
	    // 	}
	    // 	nearestVector.clear();
	    // 	nearestVector.shrink_to_fit();
	    // }
	    //Eigen::Matrix<double,CONTROL_SPACE_DIM,1> bestControl;
	    //Eigen::Matrix<double,STATE_SPACE_DIM,1> bestState;
	    Eigen::Matrix<double,STATE_SPACE_DIM,1> tempState;
	    sampleControl = RandomControl(nearest->getNodeControl(), goalControl, controlCare);
	    double bestDist;
	    bestState = spawn(nearest->getNodeState(),sampleControl);
	    bestDist = dist(bestState, sampleState);
	    //Randomly sapmle control space and pick the "best"
	    for(int i = 0; i < 50; i++) {
		sampleControl = RandomControl(nearest->getNodeControl(), goalControl, controlCare);
		tempState = spawn(nearest->getNodeState(),sampleControl);
		if(dist(tempState,sampleState) < bestDist) {
		    bestDist = dist(tempState,sampleState);
		    bestControl = sampleControl;
		    bestState = tempState;
		}
	    }
//	    if(bestDist < dist(sampleState,nearest->getNodeState()))
//	       goodSample = 1;
	}
	goodSample = 0;
	tree.push_back(new SPARSE_RRTNode(bestState, bestControl, nearest, nearest->getNodeTime() + TIME_STEP));
	if(solDist > dist(tree.back()->getNodeState(),goalState)) {
	    solDist = dist(tree.back()->getNodeState(),goalState);
	    solNode = tree.back();
	    if(solDist <= GOAL_EPSILON)
		solFound = 1;
	}
	    
	count++;
    }
    minDist = solDist;
    // Output solution distance for batch
    std::cout<<minDist<<std::endl;
    //Save the solution
    // std::vector<SPARSE_RRTNode*> solution;
    // SPARSE_RRTNode* prevNode;
    // while(solNode->getNodeParent() != NULL) {
    // 	solution.push_back(solNode);
    // 	prevNode = solNode;
    // 	solNode = solNode->getNodeParent();
    // }
    // solution.push_back(solNode);
    tree.shrink_to_fit();
    std::ofstream out_file(filename, std::ios::trunc);
    out_file <<"X Xd Y Yd Th Thd s Fn Ft P"<<std::endl;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> tempState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> tempControl;
    SPARSE_RRTNode* currentNode;
    // while(!solution.empty() && out_file.is_open()) {
    // 	currentNode = solution.back();
    // 	solution.pop_back();
    // 	tempState = currentNode->getNodeState();
    // 	tempControl = currentNode->getNodeControl();
    // 	out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
    // 		 << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
    // 	         << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0) << std::endl;
    // }

//    for(int ii = tree.length() - 1; ii >= 0; ii--} {
    SPARSE_RRTNode* currentParent;
    for(int ii = 0; ii < tree.size(); ii++) {
	currentNode = tree[ii];
	currentParent = currentNode->getNodeParent();
	if(currentParent != NULL) {
	    int j = 0;
	    for(j = 0; j < ii-1; j++) {
		if(tree[j] == currentParent)
		    break;
	    }
	    tempState = currentNode->getNodeState();
	    tempControl = currentNode->getNodeControl();
	    out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
		     << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
		     << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0)  \
		     << " " << j <<std::endl;
	} else {
	    out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
		     << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
		     << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0)  \
		     << " " << -1 <<std::endl;
	}
    }
    return 0;
}
