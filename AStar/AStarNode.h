#ifndef ASTAR_NODE_H
#define ASTAR_NODE_H

#include <stdio.h>
#include <stdlib.h>
#include <functional>
#include <array>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#define  _USE_MATH_DEFINES
#include <cmath>
#include "../Parameters.h"
#include <omp.h>


class AStarNode {
public:
    //Constructors
    AStarNode();
    AStarNode(std::array<double,STATE_SPACE_DIM> setState, std::array<double,STATE_SPACE_DIM> setGoal);
    AStarNode(AStarNode *setParent, std::array<double, STATE_SPACE_DIM> setState, \
	      std::array<double, CONTROL_SPACE_DIM> setControl, \
	      std::array<double, STATE_SPACE_DIM> setGoal, \
	      double setCost, double setTime);
    //Destructor
    // ~AStarNode();

    //Just need to get out the parameters
    AStarNode* getNodeParent();
    std::array<double, STATE_SPACE_DIM> const & getNodeState() const;
    std::array<double, CONTROL_SPACE_DIM> const & getNodeControl() const;
    double getNodePriority() const;
    double getNodeCostToCome();
    double getNodeCostToGo();
    double getNodeTime();
    AStarNode* spawn(std::array<double, CONTROL_SPACE_DIM> controlArray);

    //Return the expanded nodes
    std::vector<AStarNode*> expand();
    
protected:
    AStarNode* parent; //Where this node came from
    std::array<double, CONTROL_SPACE_DIM> nodeControl; //Array representing the control used to get here
    std::array<double, STATE_SPACE_DIM> nodeState; //Array representing the state of the node
    std::array<double, STATE_SPACE_DIM> goalState;
    double cost; //"Cost to come" of the node
    double heur; //"Cost to go" of the node
    double nodeTime;
};

struct AStarNodePtrCompare
{
    bool operator()(const AStarNode* n1, const AStarNode* n2) const
	{
	    return n1->getNodePriority() > n2->getNodePriority();
	}
};

std::array<double, CONTROL_SPACE_DIM> MapControlToWorld(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray);

double costToGo(std::array<double, STATE_SPACE_DIM> state, std::array<double, STATE_SPACE_DIM> goalState);

std::array<double, STATE_SPACE_DIM> OneStep(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> worldControl);

/* double realCost(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray, std::array<double, STATE_SPACE_DIM> goalState); */

double realCost(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray, std::array<double, STATE_SPACE_DIM> prevState);
#endif
