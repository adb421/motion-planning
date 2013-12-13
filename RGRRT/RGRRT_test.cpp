#define NDEBUG
#include <iostream>
#include <fstream>
#include <queue>
#include "RGRRTNode.h"
#include <ctime>
#include <string>

#define MAX_SAMPLES 5000

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
//    omp_set_num_threads(4);
    double distToGoal;
    //Set up the priority quee
    int discardedSamples = 0;
    std::vector<RGRRTNode*> tree;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> initState;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState;
    initState << 0, 0, 0, 0, 0, 0;
    goalState << -0.3, 0.0, 0.0, -2.0, M_PI, 0.0;
//    goalState << -0.2, 0.0, 0.1, 0.0, 0.0, 0.0;
    tree.push_back(new RGRRTNode(initState));
    bool solFound = 0;
    int count = 0;
    int good_sample;
    int onePointCount = 0;
    int twoPointCount = 0;

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
	    //For now, I'm going to assume its not.... and if it is, thats ok. I think.
	    nearestReach = nearest->getReachableStates();
//	    minDist = dist(sampleState,nearestReach[0]);
	    secondNearestReachIndex = -1;
	    for(int i = 0; i < nearestReach.size(); i++) {
		if((dist(sampleState,nearestReach[i])) < minDist) {
		    secondMinDist = minDist; //If we haven't updated yet, this is dist to neighbor
		    if(good_sample) { //Only update "second min distance" if we've already updated minDist
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
	// if(nearest->getNodeParent() == NULL)
	//     std::cout<<count<<" Root"<<std::endl;

	if(secondNearestReachIndex == -1) { //The second closest point is the nearest neighbor. Just use the nearest
	    tree.push_back(new RGRRTNode(nearestReach[nearestReachIndex], \
					 (nearest->getReachableControls())[nearestReachIndex], \
					 nearest, nearest->getNodeTime() + TIME_STEP));
	    onePointCount++;
	} else { 
	    //Get the controls
	    //First find distance to closest point on the line
	    Eigen::Matrix<double,STATE_SPACE_DIM,1> P1 = nearestReach[nearestReachIndex];
	    Eigen::Matrix<double,STATE_SPACE_DIM,1> P2 = nearestReach[secondNearestReachIndex];
	    double lineLength = sqrt(((P2 - P1).transpose()*(P2-P1))(0,0));
	    //P is sampleState
	    //Unit length along line.
	    double distAlongLine = (((sampleState - P1).transpose())*(P2-P1)/lineLength/lineLength)(0,0);
	    //If its outside the line, we can't use it.
	    // if(distAlongLine > 1.0)
	    // 	distAlongLine = 1.0;
	    // else if(distAlongLine < 0.0)
	    // 	distAlongLine = 0.0;

	    //Now figure out how far along the control you go.
	    Eigen::Matrix<double,CONTROL_SPACE_DIM,1> C1 = (nearest->getReachableControls())[nearestReachIndex];
	    Eigen::Matrix<double,CONTROL_SPACE_DIM,1> C2 = \
		(nearest->getReachableControls())[secondNearestReachIndex];
	    Eigen::Matrix<double,CONTROL_SPACE_DIM,1> CUse = C1 + distAlongLine*(C2-C1);
	    if(CUse(0,0) < 0.0)
		CUse(0,0) = 0.0;
	    else if(CUse(0,0) > 2.0*LO)
		CUse(0,0) = 2.0*LO;
	    if(CUse(1,0) < 0.0)
		CUse(1,0) = 0.0;
	    else if(CUse(1,0) > MAX_FN)
		CUse(1,0) = MAX_FN;
	    if(CUse(2,0) < -MU*CUse(1,0))
		CUse(2,0) = -MU*CUse(1,0);
	    else if(CUse(2,0) > MU*CUse(1,0))
		CUse(2,0) = MU*CUse(1,0);
	    tree.push_back(new RGRRTNode(spawn(nearest->getNodeState(),CUse), \
					 CUse, nearest, nearest->getNodeTime() + TIME_STEP));
	    twoPointCount++;
	}
	    //check if its a solution
	    //If not, new sample
	count++;
	// if(count % 100 == 0)
	//     std::cout << "Step: " << count << std::endl;
	//Check if the node is close enough
	if(dist(goalState,tree.back()->getNodeState()) <= GOAL_EPSILON) {
//	    std::cout << "Done!" << std::endl;
	    solFound = 1;
	}
    }
//    std::cout << "Tree has " << tree.size() << " nodes." << std::endl;
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
//	if(solNode == tree[0])
//	    solNode = tree.back();
    } else {
	solNode = tree.back();
    }
    // std::cout<<"Solution is at a distance of " << minDist << " from the goal." << std::endl;
    // std::cout<<"Final State:" << std::endl << solNode->getNodeState() << std::endl;
    // std::cout<<"Goal State:" << std::endl << goalState << std::endl;
    // Output solution distance for batch
    std::cout<<minDist<<std::endl;
    //Save the solution
    std::vector<RGRRTNode*> solution;
    RGRRTNode* prevNode;
    while(solNode->getNodeParent() != NULL) {
	solution.push_back(solNode);
	prevNode = solNode;
	solNode = solNode->getNodeParent();
    }
    solution.push_back(solNode);
    // std::cout << "Discarded " << discardedSamples << " samples." <<std::endl;
    // std::cout << "One Point Samples: " << onePointCount << std::endl;
    // std::cout << "Two Point Samples: " << twoPointCount << std::endl;
    // std::cout << "Solution has " << solution.size() << " nodes" << std::endl;
    // std::cout << "Outputting files" << std::endl;
    std::ofstream out_file(filename, std::ios::trunc);
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
    // std::cout << "Deleting nodes" << std::endl;
    // //Delete!!!! openQueue and closedNodes
    // while(!tree.empty()) {
    // 	delete tree.back();
    // 	tree.pop_back();
    // }

//    std::cout<<"Time: " << (std::clock() - start)/(double)CLOCKS_PER_SEC<<std::endl;
    return 0;
}
