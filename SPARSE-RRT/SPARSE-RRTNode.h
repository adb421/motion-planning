#ifndef SPARSE_RRT_NODE_H
#define SPARSE_RRT_NODE_H

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
#include <map>

typedef Eigen::Matrix<double, STATE_SPACE_DIM,1> StateVector_t;
typedef Eigen::Matrix<double, CONTROL_SPACE_DIM,1> ControlVector_t;

class SPARSE_RRTNode {
public:
    //Constructors
    SPARSE_RRTNode();
    SPARSE_RRTNode(StateVector_t setState);
    SPARSE_RRTNode(StateVector_t setState, \
	      ControlVector_t setControl, \
	      SPARSE_RRTNode* setParent, double setTime);
    
    //Destructor
    // ~SPARSE_RRTNode();

    //Get out parameters
    ControlVector_t const & getNodeControl() const;
    StateVector_t const & getNodeState() const;
    SPARSE_RRTNode* getNodeParent();
    double getNodeTime();
    std::array<ControlVector_t, NUM_DISC> getReachableControls();

protected:
    ControlVector_t nodeControl; //Array representing the control used to get here
    StateVector_t nodeState; //Array representing the state of the node
//    std::vector<SPARSE_RRTNode*> nodeChildren; //Vector representing the children
    SPARSE_RRTNode* parent; //Reperesents the parent node
    double nodeTime;
};

typedef std::map<std::array<int, STATE_SPACE_DIM>, SPARSE_RRTNode*> map_t;

ControlVector_t MapControlToWorld(StateVector_t state, \
							     ControlVector_t controlArray);

StateVector_t OneStep(Eigen::Matrix<double, STATE_SPACE_DIM, 1> state, \
						 ControlVector_t worldControl);

StateVector_t spawn(StateVector_t state, \
					       ControlVector_t worldControl);

double dist(StateVector_t state1, StateVector_t state2);

StateVector_t RandomSample(Eigen::Matrix<double,STATE_SPACE_DIM,1> goal);

ControlVector_t RandomControl(ControlVector_t controlMean, ControlVector_t controlGoal, std::array<int,CONTROL_SPACE_DIM> controlCare);

std::array<int, STATE_SPACE_DIM> snapToGrid(SPARSE_RRTNode* nodeToSnap);

int violateConstraints(SPARSE_RRTNode* checkNode);

double minTimePoly(std::array<double,5> coeffs);

std::array<double,5> getMinTimeCoefficients(StateVector_t x0, StateVector_t x1);

#endif
