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
#include "AStar_Parameters.h"
#include <omp.h>
#include <complex>
#include <map>
#include <utility>

void printBadCount();

class AStarNode;

typedef Eigen::Matrix<double, STATE_SPACE_DIM,1> StateVector_t;
typedef Eigen::Matrix<double, CONTROL_SPACE_DIM,1> ControlVector_t;
typedef std::map<std::array<int, STATE_SPACE_DIM>, AStarNode*> map_t;

class AStarNode {
public:
    //Constructors
    AStarNode();
    AStarNode(StateVector_t setState, StateVector_t setGoal);
    AStarNode(AStarNode *setParent, StateVector_t setState, \
	      ControlVector_t setControl, \
	      StateVector_t setGoal, \
	      double setCost, double setTime);
    //Destructor
    // ~AStarNode();

    //Just need to get out the parameters
    AStarNode* getNodeParent();
    StateVector_t const & getNodeState() const;
    ControlVector_t const & getNodeControl() const;
    double getNodePriority() const;
    double getNodeCostToCome();
    double getNodeCostToGo();
    double getNodeTime();
    AStarNode* spawn(ControlVector_t controlArray);
    bool good;

    //Return the expanded nodes
    std::vector<AStarNode*> expand(map_t  &grid);
    
protected:
    AStarNode* parent; //Where this node came from
    ControlVector_t nodeControl; //Array representing the control used to get here
    StateVector_t nodeState; //Array representing the state of the node
    StateVector_t goalState;
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

std::array<int, STATE_SPACE_DIM> snapToGrid(AStarNode* nodeToSnap);

ControlVector_t MapControlToWorld(StateVector_t state, ControlVector_t controlArray);

double costToGo(StateVector_t state, StateVector_t goalState);

StateVector_t OneStep(StateVector_t state, ControlVector_t worldControl);

/* double realCost(StateVector_t state, ControlVector_t controlArray, StateVector_t goalState); */

double realCost(StateVector_t state, ControlVector_t controlArray, StateVector_t prevState, StateVector_t goalState);

Eigen::VectorXcd polynomialRoots(std::array<double,5> coeffs);
//void polynomialRoots(std::array<double,5> coeffs);

double minTimePoly(std::array<double,5> coeffs);

int violateConstraints(AStarNode* checkNode);

std::array<double,5> getMinTimeCoefficients(StateVector_t x0, StateVector_t x1);

double euclideanDistance(StateVector_t state1, StateVector_t state2);
    
#endif
