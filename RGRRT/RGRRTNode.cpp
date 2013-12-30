#include "RGRRTNode.h"

//Weighting matrix for distance metric
double QDist_array[STATE_SPACE_DIM*STATE_SPACE_DIM] =			\
{1/pow((MAX_X - MIN_X),2), 0,   0, 0, 0, 0,				\
 0,      1/pow((MAXVEL_XY*2),2), 0, 0, 0, 0,				\
 0,       0,   1/pow((MAX_Y - MIN_Y),2), 0, 0, 0,			\
 0,       0,   0,      1/pow((MAXVEL_XY*2),2), 0, 0,			\
 0,       0,   0,      0,   1/pow((MAX_TH - MIN_TH),2), 0,		\
 0,       0,   0,      0,   0,      1/pow((MAXVEL_TH),2)};

Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> QDist = \
    Eigen::Map<Eigen::MatrixXd>(QDist_array, STATE_SPACE_DIM, STATE_SPACE_DIM);

RGRRTNode::RGRRTNode() {
    parent = NULL;
    nodeTime = 0;
    for(int i = 0; i < STATE_SPACE_DIM; i++)
	nodeState(i,0) = 0;
    for(int i = 0; i < CONTROL_SPACE_DIM; i++)
	nodeControl(i,0) = 0;
    this->FindReachableSet();
}

RGRRTNode::RGRRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState) {
    parent = NULL;
    nodeTime = 0;
    nodeState = setState;
    for(int i = 0; i < CONTROL_SPACE_DIM; i++)
	nodeControl(i,0) = 0;
    this->FindReachableSet();
}

RGRRTNode::RGRRTNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState,	\
	      Eigen::Matrix<double, CONTROL_SPACE_DIM,1> setControl, \
	  RGRRTNode* setParent, double setTime){
    parent = setParent;
    nodeTime = setTime;
    nodeState = setState;
    nodeControl = setControl;
    this->FindReachableSet();
}

void RGRRTNode::FindReachableSet(){
    //Need to discretize control space, and fill in reachableStates and reachableControls arrays
    Eigen::ArrayXd contactPointLocations = Eigen::ArrayXd::LinSpaced(DISC_S,-LO,LO);
    Eigen::ArrayXd normalForceMagnitude = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd tangentForceMagnitude = Eigen::ArrayXd::LinSpaced(DISC_FT,-1.0*MU,MU);
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray;
    int counter = 0;
    for(int i = 0; i < contactPointLocations.rows(); i++) {
	for(int j = 0; j < normalForceMagnitude.rows(); j++) {
	    for(int k = 0; k < tangentForceMagnitude.rows(); k++) {
		controlArray << contactPointLocations(i,0), \
		    normalForceMagnitude(j,0),		    \
		    tangentForceMagnitude(k,0)*normalForceMagnitude(j,0);
		reachableStates[counter] = spawn(nodeState,controlArray);
		reachableControls[counter] = controlArray;
		counter++;
	    }
	}
    }
}

Eigen::Matrix<double, CONTROL_SPACE_DIM,1> const & RGRRTNode::getNodeControl() const {
    return nodeControl;
}

Eigen::Matrix<double, STATE_SPACE_DIM,1> const & RGRRTNode::getNodeState() const {
    return nodeState;
}

double RGRRTNode::getNodeTime() {
    return nodeTime;
}

std::array<Eigen::Matrix<double, STATE_SPACE_DIM,1>, NUM_DISC> RGRRTNode::getReachableStates(){
    return reachableStates;
}

std::array<Eigen::Matrix<double, CONTROL_SPACE_DIM,1>, NUM_DISC> RGRRTNode::getReachableControls(){
    return reachableControls;
}

RGRRTNode* RGRRTNode::getNodeParent(){
    return parent;
}

Eigen::Matrix<double, CONTROL_SPACE_DIM,1>				\
MapControlToWorld(Eigen::Matrix<double, STATE_SPACE_DIM,1> state,	\
		  Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray) {
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl;
    double conPoint = controlArray(0,0);
    double Fn = controlArray(1,0);
    double Ft = controlArray(2,0);
    double theta = state(4,0);
    worldControl(0,0) = Ft*cos(theta) - Fn*sin(theta);
    worldControl(1,0) = Fn*cos(theta) + Ft*sin(theta);
    worldControl(2,0) = WO*Ft + conPoint*Fn;
    return worldControl;
}

Eigen::Matrix<double, STATE_SPACE_DIM,1> OneStep(Eigen::Matrix<double, STATE_SPACE_DIM, 1> state, \
						 Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl) {
    Eigen::Matrix<double, STATE_SPACE_DIM,1> derivState;
    derivState << state(1,0),
	worldControl(0,0)/MO,
	state(3,0),
	worldControl(1,0)/MO - GRAV,
	state(5,0),
	worldControl(2,0)/JO;
    state = state + derivState*INT_TIME_STEP;
    return state;
}

Eigen::Matrix<double, STATE_SPACE_DIM,1> spawn(Eigen::Matrix<double, STATE_SPACE_DIM, 1> state, \
					       Eigen::Matrix<double, CONTROL_SPACE_DIM,1> control) {
    double time = 0;
    while(time <= TIME_STEP) {
	state = OneStep(state, MapControlToWorld(state,control));
	time += INT_TIME_STEP;
    }
    return state;
}

Eigen::Matrix<double, STATE_SPACE_DIM,1> RandomSample(Eigen::Matrix<double,STATE_SPACE_DIM,1> goal) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0,1);
    if(dis(gen) < BIAS) {
	return goal;
    }
    //Not biased anymore
    Eigen::Matrix<double,STATE_SPACE_DIM,1> samp;
    samp << (MIN_X + (MAX_X - MIN_X)*(dis(gen))),	\
	MAXVEL_XY*2*(dis(gen) - 0.5),			\
	(MIN_Y + (MAX_Y - MIN_Y)*(dis(gen))),		\
	MAXVEL_XY*2*(dis(gen) - 0.5),			\
	(MIN_TH + (MAX_TH - MIN_TH)*(dis(gen))),	\
	MAXVEL_TH*2*(dis(gen) - 0.5);
    return samp;
}

double dist(Eigen::Matrix<double,STATE_SPACE_DIM,1> state1, Eigen::Matrix<double,STATE_SPACE_DIM,1> state2) {
    return (((state1 - state2).transpose())*QDist*(state1-state2))(0,0);
//    return (((state1 - state2).transpose())*(state1-state2))(0,0);
}


// //Might be TOTALLY UNNECESSSARY!
// //Create the convex hull of the reachable points
// Convex_hull_d* createConvexHullReachable(std::array<std::array<double,STATE_SPACE_DIM>,NUM_DISC> reachable_states) {
//     //Create an array of Point_d's, use that to make the convex hull?
//     std::vector<Point_d> points;
//     for(int i = 0; i < reachable_states.size(); i++) {
// 	points.push_back(Point_d(STATE_SPACE_DIM, reachable_states[i].begin(), reachable_states[i].end()));
//     }
//     Convex_hull_d* con_hull = new Convex_hull_d(6);
// //    CGAL::Convex_hull_d(points.begin(), points.end(), poly);
//     std::vector<Point_d>::iterator it;
//     for(it = points.begin(); it!= points.end(); ++it) {
// 	con_hull->insert(*it);
//     }
//     con_hull->is_valid(true);
//     return con_hull;
// }
