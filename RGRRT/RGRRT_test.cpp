#define NDEBUG
#include <iostream>
#include <fstream>
#include <queue>
#include "RGRRTNode.h"
#include <ctime>

#define MAX_SAMPLES 20000

int main(int argc, char** argv)
{
    clock_t start;
    start = std::clock();
//    omp_set_num_threads(4);
    double distToGoal;
    //Set up the priority quee
    int discardedSamples = 0;
    std::vector<RGRRTNode*> tree;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> initState;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState;
    initState << 0, 0, 0, 0, 0, 0;
//    goalState << -0.3, 0.0, 0.0, -1.75, M_PI, 0.0;
    goalState << -0.2, 0.0, 0.1, 0.0, 0.0, 0.0;
    tree.push_back(new RGRRTNode(initState));
    bool solFound = 0;
    int count = 0;
    int good_sample;

    Eigen::Matrix<double, STATE_SPACE_DIM,1> sampleState;
    RGRRTNode* nearest;
    double minDist;
    double secondMinDist = -1;
    std::array<Eigen::Matrix<double, STATE_SPACE_DIM,1>, NUM_DISC> nearestReach;
    int nearestReachIndex;
    int secondNearestReachIndex;
    
    while(!solFound && count < MAX_SAMPLES)
    {
	good_sample = 0;
	while(!good_sample) {
	    //sample a random state
	    sampleState = RandomSample(goalState);
	    //Find its nearest neighbor in the tree
	    nearest = tree[0];
	    minDist = dist(sampleState,nearest->getNodeState());
	    secondMinDist = -1; //Reset
	    for(int i = 1; i < tree.size(); i++) {
		if((dist(sampleState,tree[i]->getNodeState())) < minDist) {
		    minDist = dist(sampleState,tree[i]->getNodeState());
		    nearest = tree[i];
		}
	    }
	    //This will only work for points outside the hull
	    //We should check if the sample is INSIDE the convex hull first
	    nearestReach = nearest->getReachableStates();
//	    minDist = dist(sampleState,nearestReach[0]);
	    secondNearestReachIndex = -1;
	    for(int i = 0; i < nearestReach.size(); i++) {
		if((dist(sampleState,nearestReach[i])) < minDist) {
		    secondMinDist = minDist;
		    if(!good_sample) { //Only update "second min distance" if we've already updated minDist
			secondNearestReachIndex = nearestReachIndex;
		    }
		    good_sample = 1;
		    nearestReachIndex = i;
		    minDist = dist(sampleState,nearestReach[i]);
		} else {
		    if(dist(sampleState,nearestReach[i]) < secondMinDist) { //Impossible if 2ndmindist = -1
			secondNearestReachIndex = i;
			secondMinDist = dist(sampleState,nearestReach[i]);
		    }
		}
	    }
	    //Check good sample
	    if(!good_sample)
		discardedSamples++;
	}

	if(secondNearestReachindex == -1) { //The second closest point is the nearest neighbor. Just use the nearest
	    
	}
	tree.push_back(new RGRRTNode(nearestReach[nearestReachIndex],	\
				     (nearest->getReachableControls())[nearestReachIndex], \
				     nearest, nearest->getNodeTime() + TIME_STEP));
	    //check if its a solution
	    //If not, new sample
	count++;
	if(count % 100 == 0)
	    std::cout << "Step: " << count << std::endl;
	//Check if the node is close enough
	if(dist(goalState,tree.back()->getNodeState()) <= GOAL_EPSILON) {
	    std::cout << "Done!" << std::endl;
	    solFound = 1;
	}
    }
    std::cout << "Tree has " << tree.size() << " nodes." << std::endl;
    RGRRTNode* solNode;
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
    std::cout<<"Solution is at a distance of " << minDist << " from the goal." << std::endl;
    std::cout<<"Final State:" << std::endl << solNode->getNodeState() << std::endl;
    std::cout<<"Goal State:" << std::endl << goalState << std::endl;
    //Save the solution
    std::vector<RGRRTNode*> solution;
    RGRRTNode* prevNode;
    while(solNode->getNodeParent() != NULL) {
	solution.push_back(solNode);
	prevNode = solNode;
	solNode = solNode->getNodeParent();
	if(solNode == prevNode) {
	    std::cout << "weird pointer behavior" << std::endl;
	    break;
	}
    }
    solution.push_back(solNode);
    std::cout << "Discarded " << discardedSamples << " samples." <<std::endl;
    std::cout << "Solution has " << solution.size() << " nodes" << std::endl;
    std::cout << "Outputting files" << std::endl;
    std::ofstream out_file("Solution.txt", std::ios::trunc);
    out_file <<"X Xd Y Yd Th Thd s Fn Ft"<<std::endl;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> tempState;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> tempControl;
    RGRRTNode* currentNode;
    while(!solution.empty() && out_file.is_open()) {
	currentNode = solution.back();
	solution.pop_back();
	tempState = currentNode->getNodeState();
	tempControl = currentNode->getNodeControl();
	out_file << tempState(0,0) << " " << tempState(1,0) << " " << tempState(2,0) << " " \
		 << tempState(3,0) << " " << tempState(4,0) << " " << tempState(5,0) << " " \
	         << tempControl(0,0) << " " << tempControl(1,0) << " " << tempControl(2,0) << std::endl;
    }
    std::cout << "Deleting nodes" << std::endl;
    //Delete!!!! openQueue and closedNodes
    while(!tree.empty()) {
	delete tree.back();
	tree.pop_back();
    }

    std::cout<<"Time: " << (std::clock() - start)/(double)CLOCKS_PER_SEC<<std::endl;
    return 0;
}
