#ifndef RGRRT_NODE_H
#define RGRRT_NODE_H

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
//#include <CGAL/Cartesian.h>
//#include <CGAL/Cartesian_d.h>
//#include <CGAL/Cartesian.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/Kernel_d/Point_d.h>
#include <CGAL/Homogeneous_d.h>
#include <CGAL/Homogeneous.h>
#include <CGAL/Convex_hull_d.h>
#include <CGAL/double.h>

//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef double RT;
//typedef CGAL::Cartesian_d<RT> Kernel;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Homogeneous_d<RT> Kernel;
typedef CGAL::Convex_hull_d<Kernel> Convex_hull_d;
typedef Convex_hull_d::Point_d Point_d;
typedef Convex_hull_d::Simplex_handle Simplex_handle;


class RGRRTNode {
public:
    //Constructors
    RGRRTNode();
    RGRRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState);
    RGRRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState, \
	      Eigen::Matrix<double, CONTROL_SPACE_DIM,1> setControl, \
	      RGRRTNode* setParent, double setTime);
    
    //Destructor
    // ~RGRRTNode();

    //Get out parameters
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> const & getNodeControl() const;
    Eigen::Matrix<double, STATE_SPACE_DIM,1> const & getNodeState() const;
    RGRRTNode* getNodeParent();
    double getNodeTime();
    std::array<Eigen::Matrix<double, STATE_SPACE_DIM,1>, NUM_DISC> getReachableStates();
    std::array<Eigen::Matrix<double, CONTROL_SPACE_DIM,1>, NUM_DISC> getReachableControls();

protected:
    //Find reachable set
    void FindReachableSet();
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> nodeControl; //Array representing the control used to get here
    Eigen::Matrix<double, STATE_SPACE_DIM,1> nodeState; //Array representing the state of the node
//    std::vector<RGRRTNode*> nodeChildren; //Vector representing the chil
    RGRRTNode* parent; //Reperesents the parent node
    std::array<Eigen::Matrix<double, STATE_SPACE_DIM,1>, NUM_DISC> reachableStates;
    std::array<Eigen::Matrix<double, CONTROL_SPACE_DIM,1>, NUM_DISC> reachableControls;
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

Convex_hull_d* createConvexHullReachable(std::array<std::array<double,STATE_SPACE_DIM>,NUM_DISC> reachable_states);

#endif
