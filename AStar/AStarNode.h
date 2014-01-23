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
#include "Parameters.h"
#include <omp.h>


class AStarNode {
public:
    //Constructors
    AStarNode();
    AStarNode(Eigen::Matrix<double,STATE_SPACE_DIM,1> setState, Eigen::Matrix<double,STATE_SPACE_DIM,1> setGoal);
    AStarNode(AStarNode *setParent, Eigen::Matrix<double, STATE_SPACE_DIM,1> setState, \
	      Eigen::Matrix<double, CONTROL_SPACE_DIM,1> setControl, \
	      Eigen::Matrix<double, STATE_SPACE_DIM,1> setGoal, \
	      double setCost, double setTime);
    //Destructor
    // ~AStarNode();

    //Just need to get out the parameters
    AStarNode* getNodeParent();
    Eigen::Matrix<double, STATE_SPACE_DIM,1> const & getNodeState() const;
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> const & getNodeControl() const;
    double getNodePriority() const;
    double getNodeCostToCome();
    double getNodeCostToGo();
    double getNodeTime();
    AStarNode* spawn(Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray);

    //Return the expanded nodes
    std::vector<AStarNode*> expand();
    
protected:
    AStarNode* parent; //Where this node came from
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> nodeControl; //Array representing the control used to get here
    Eigen::Matrix<double, STATE_SPACE_DIM,1> nodeState; //Array representing the state of the node
    Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState;
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

Eigen::Matrix<double, CONTROL_SPACE_DIM,1> MapControlToWorld(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray);

double costToGo(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState);

Eigen::Matrix<double, STATE_SPACE_DIM,1> OneStep(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl);

/* double realCost(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray, Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState); */

double realCost(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray, Eigen::Matrix<double, STATE_SPACE_DIM,1> prevState, Eigen::Matrix<double, STATE_SPACE_DIM,1> goalState);


#endif
