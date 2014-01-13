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

class SPARSE_RRTNode {
public:
    //Constructors
    SPARSE_RRTNode();
    SPARSE_RRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState);
    SPARSE_RRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState, \
	      Eigen::Matrix<double, CONTROL_SPACE_DIM,1> setControl, \
	      SPARSE_RRTNode* setParent, double setTime);
    
    //Destructor
    // ~SPARSE_RRTNode();

    //Get out parameters
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> const & getNodeControl() const;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> const & getNodeState() const;
    SPARSE_RRTNode* getNodeParent();
    double getNodeTime();
    std::array<Eigen::Matrix<double, CONTROL_SPACE_DIM,1>, NUM_DISC> getReachableControls();

protected:
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> nodeControl; //Array representing the control used to get here
    Eigen::Matrix<double, STATE_SPACE_DIM,1> nodeState; //Array representing the state of the node
//    std::vector<SPARSE_RRTNode*> nodeChildren; //Vector representing the children
    SPARSE_RRTNode* parent; //Reperesents the parent node
    double nodeTime;
};

Eigen::Matrix<double, CONTROL_SPACE_DIM,1> MapControlToWorld(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, \
							     Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray);

Eigen::Matrix<double, STATE_SPACE_DIM,1> OneStep(Eigen::Matrix<double, STATE_SPACE_DIM, 1> state, \
						 Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl);

Eigen::Matrix<double, STATE_SPACE_DIM,1> spawn(Eigen::Matrix<double, STATE_SPACE_DIM,1> state, \
					       Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl);

double dist(Eigen::Matrix<double,STATE_SPACE_DIM,1> state1, Eigen::Matrix<double,STATE_SPACE_DIM,1> state2);

Eigen::Matrix<double, STATE_SPACE_DIM,1> RandomSample(Eigen::Matrix<double,STATE_SPACE_DIM,1> goal);

Eigen::Matrix<double, CONTROL_SPACE_DIM,1> RandomControl(Eigen::Matrix<double, CONTROL_SPACE_DIM,1>);


#endif
