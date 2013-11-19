#include "AStarNode.h"

// double Qf_array[36] = {50000.0, 0,   0, 0, 0, 0, \
// 		      0,       4.0, 0, 0, 0, 0, \
// 		      0,       0,   10000.0, 0, 0, 0, \
// 		      0,       0,   0,      4.0, 0, 0, \
// 		      0,       0,   0,      0,   1000.0, 0, \
// 		      0,       0,   0,      0,   0,      4.0};
double Qf_array[STATE_SPACE_DIM*STATE_SPACE_DIM] =			\
{QC/pow((MAX_X - MIN_X),2), 0,   0, 0, 0, 0,				\
 0,      QC/pow((MAXVEL_XY*2),2), 0, 0, 0, 0,				\
 0,       0,   QC/pow((MAX_Y - MIN_Y),2), 0, 0, 0,			\
 0,       0,   0,      QC/pow((MAXVEL_XY*2),2), 0, 0,			\
 0,       0,   0,      0,   QC/pow((MAX_TH - MIN_TH),2), 0,		\
 0,       0,   0,      0,   0,      QC/pow((MAXVEL_TH),2)};

double R_array[9] = {1.0, 0,   0, \
		     0,   0.001, 0, \
		     0,   0,   10.0};
Eigen::Matrix<double, 6, 6> Qf = Eigen::Map<Eigen::MatrixXd>(Qf_array, 6, 6);
Eigen::Matrix<double, 3, 3> Rf = Eigen::Map<Eigen::MatrixXd>(R_array, 3, 3);

//Default Constructor
//Assume root of tree with zero initial state
AStarNode::AStarNode() {
    parent = NULL;
    int j;
    for(j = 0; j < STATE_SPACE_DIM; j++)
    {
	nodeState[j] = 0.0;
	goalState[j] = 0.0;
    }
    for(j = 0; j < CONTROL_SPACE_DIM; j++)
	nodeControl[j] = 0.0;
    cost = 0.0; heur = 0.0; nodeTime = 0.0;
}

//Constructor for root
AStarNode::AStarNode(std::array<double, STATE_SPACE_DIM> setState, std::array<double, STATE_SPACE_DIM> setGoal) {
    parent = NULL;
    nodeState = setState;
    goalState = setGoal;
    int j;
    for(j = 0; j < CONTROL_SPACE_DIM; j++)
	nodeControl[j] = 0.0;
    cost = 0.0;
    nodeTime = 0.0;
    heur = costToGo(nodeState, goalState);
}

//Constructor for most nodes
AStarNode::AStarNode(AStarNode *setParent, std::array<double, STATE_SPACE_DIM> setState, \
		     std::array<double, CONTROL_SPACE_DIM> setControl, \
		     std::array<double, STATE_SPACE_DIM> setGoal, \
		     double setCost, double setTime) {
    parent = setParent;
    nodeState = setState;
    goalState = setGoal;
    nodeControl = setControl;
    nodeTime = setTime;
    cost = setCost;
    heur = costToGo(nodeState, goalState);
}

//Return a pointer to the parent
AStarNode* AStarNode::getNodeParent()
{
    return parent;
}

//Return the state contained by this node
std::array<double, STATE_SPACE_DIM> const & AStarNode::getNodeState() const
{
    return nodeState;
}

//Return the control used to get to the node
std::array<double, CONTROL_SPACE_DIM> const & AStarNode::getNodeControl() const
{
    return nodeControl;
}

//Return the node's priority (cost + heuristic)
double AStarNode::getNodePriority() const
{
    return cost + heur;
}

double AStarNode::getNodeTime() {
    return nodeTime;
}

//Return the cost to come to this point from the root along this specified path
double AStarNode::getNodeCostToCome()
{
    return cost;
}

//Return how far we are from the goal according to the heuristic
double AStarNode::getNodeCostToGo()
{
    return heur;
}

//Expand the node, return all child nodes as a vector
std::vector<AStarNode*> AStarNode::expand()
{
    double FnMin;
    //TODO: Incorporate length of the side
    Eigen::ArrayXd contactPointLocations = Eigen::ArrayXd::LinSpaced(DISC_S,0,2*LO);
    Eigen::ArrayXd normalForceMagnitude = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd tangentForceMagnitude;
    std::array<double, CONTROL_SPACE_DIM> controlArray;
    std::vector<AStarNode*> childNodes;//(DISC_S*DISC_FN*DISC_FT);
    std::vector<AStarNode*> tempChildren;
    AStarNode* tempNode;

    int i, j, k;
//     omp_set_num_threads(4);
// #pragma omp parallel for						\
//     shared(i,j,k,childNodes,contactPointLocations,normalForceMagnitude) \
//     schedule(static) 							\
//    private(controlArray,tangentForceMagnitude,tempChildren,tempNode) collapse(2)
    for(i =  0; i < contactPointLocations.size(); i++)
    {
	for(j = 0; j < normalForceMagnitude.size(); j++)
	{
	    //Initialize friction force array
	    tangentForceMagnitude = Eigen::ArrayXd::LinSpaced(DISC_FT,-MU*normalForceMagnitude[j],MU*normalForceMagnitude[j]);
	    for(k = 0; k < tangentForceMagnitude.size(); k++)
	    {
		controlArray[0] = contactPointLocations[i];
		controlArray[1] = normalForceMagnitude[j];
		controlArray[2] = tangentForceMagnitude[k];
		tempNode = this->spawn(controlArray);
		tempChildren.push_back(tempNode);
	    }
//#pragma omp critical
	    {
		for(int it = 0; it < tempChildren.size(); it++) {
		    childNodes.push_back(tempChildren.back());
		    tempChildren.pop_back();
		}
	    }
	}
    }
    return childNodes;
}

//#define TIME_COST 1
AStarNode* AStarNode::spawn(std::array<double, CONTROL_SPACE_DIM> controlArray) {
    std::array<double, STATE_SPACE_DIM> state = nodeState;
    std::array<double, STATE_SPACE_DIM> prevState;
    std::array<double, CONTROL_SPACE_DIM> worldControl;
    double time = nodeTime;
    double newCost = cost;
    while(time <= TIME_STEP + nodeTime) {
	prevState = state;
	//Map controls from normal/tangent/contact point to world x/y/theta
	worldControl = MapControlToWorld(state, controlArray);
	//Euler integrate
	state = OneStep(state, worldControl);
	//Increment cost
	newCost += realCost(state, controlArray, prevState);
//	newCost += realCost(state, controlArray, goalState);
	time += INT_TIME_STEP;
    }
    return new AStarNode(this, state, controlArray, goalState, newCost, time);
}

//Contact, normal, tangent - control
//xo, xod, yo, yod, tho, thod - state
std::array<double, CONTROL_SPACE_DIM> MapControlToWorld(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray) {
    std::array<double, CONTROL_SPACE_DIM> worldControl;
    double conPoint = controlArray[0];
    double Fn = controlArray[1];
    double Ft = controlArray[2];
    double theta = state[4];
    worldControl[0] = Ft*cos(theta) - Fn*sin(theta);
    worldControl[1] = Fn*cos(theta) + Ft*sin(theta);
    worldControl[2] = WO*Ft + (conPoint - LO)*Fn;
    return worldControl;
}

double costToGo(std::array<double, STATE_SPACE_DIM> state, std::array<double, STATE_SPACE_DIM> goalState)
{
    Eigen::Matrix<double, STATE_SPACE_DIM, 1> stateVec = Eigen::Map<Eigen::MatrixXd>(state.data(), 6, 1);
    Eigen::Matrix<double, STATE_SPACE_DIM, 1> goalVec = Eigen::Map<Eigen::MatrixXd>(goalState.data(), 6, 1);
    Eigen::Matrix<double, STATE_SPACE_DIM, 1> errorVec = stateVec - goalVec;
    //Quadruple cost if y is below when y velocity is the same
    if(goalVec(1,0) != 0 && std::signbit(errorVec(0,0)) == std::signbit(goalVec(1,0)))
       errorVec(0,0) *= 2;
    if(goalVec(3,0) != 0 && std::signbit(errorVec(2,0)) == std::signbit(goalVec(3,0)))
	errorVec(2,0) *= 2;
    if(goalVec(5,0) != 0 && std::signbit(errorVec(3,0)) == std::signbit(goalVec(5,0)))
	errorVec(3,0) *= 2;
    return ((errorVec.transpose())*Qf*errorVec)(0,0);
}

std::array<double, STATE_SPACE_DIM> OneStep(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> worldControl) {

    std::array<double, STATE_SPACE_DIM> derivState = {state[1], worldControl[0]/MO, \
						      state[3], worldControl[1]/MO - GRAV, \
						      state[5], worldControl[2]/JO};
    for(int i = 0; i < STATE_SPACE_DIM; i++) {
	if(!BACKWARDS_INT)
	    state[i] += derivState[i]*INT_TIME_STEP;
	else
	    state[i] -= derivState[i]*INT_TIME_STEP;
    }
    return state;
}

// double realCost(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray, std::array<double, STATE_SPACE_DIM> goalState) {
//     Eigen::Matrix<double, 6, 1> stateVec = Eigen::Map<Eigen::MatrixXd>(state.data(), 6, 1);
//     Eigen::Matrix<double, 6, 1> goalVec = Eigen::Map<Eigen::MatrixXd>(goalState.data(), 6, 1);
//     Eigen::Matrix<double, 3, 1> controlVec = Eigen::Map<Eigen::MatrixXd>(controlArray.data(), 3, 1);
//     Eigen::Matrix<double, 3, 1> desControl;
//     desControl << LO, 0, 0;
// //    return ((((goalVec - stateVec).transpose())*Qf*(goalVec - stateVec))(0,0) + (((desControl - controlVec).transpose())*Rf*(desControl - controlVec))(0,0))*INT_TIME_STEP;
//     return ((((desControl - controlVec).transpose())*Rf*(desControl - controlVec))(0,0) + 1)*INT_TIME_STEP;
// }

double realCost(std::array<double, STATE_SPACE_DIM> state, std::array<double, CONTROL_SPACE_DIM> controlArray, std::array<double, STATE_SPACE_DIM> prevState) {
    double prevTh = prevState[4];
    double prevXVel = prevState[1];
    double prevYVel = prevState[3];
    double prevThVel = prevState[5];
    Eigen::Vector3d prevBodyVel(prevXVel*cos(prevTh) + prevYVel*sin(prevTh), \
				prevYVel*cos(prevTh) - prevXVel*sin(prevTh), \
				prevThVel);
    if(prevBodyVel.isZero())
	prevBodyVel << 0.0, 1.0, 0.0;
    double Th = state[4];
    double XVel = state[1];
    double YVel = state[3];
    double ThVel = state[5];
    Eigen::Vector3d bodyVel(XVel*cos(Th) + YVel*sin(Th), \
			    YVel*cos(Th) - XVel*sin(Th), \
			    ThVel);

    Eigen::Matrix<double, CONTROL_SPACE_DIM, 1> controlVec = Eigen::Map<Eigen::MatrixXd>(controlArray.data(), CONTROL_SPACE_DIM, 1);
    Eigen::Matrix<double, CONTROL_SPACE_DIM, 1> desControl;
    desControl << LO, 0, 0;
    double temp = (VELSCALE*abs((prevBodyVel.cross(bodyVel)/(prevBodyVel.norm()*bodyVel.norm()))(0,0)) + \
	 ((controlVec - desControl).transpose()*Rf*(controlVec - desControl))(0,0)) * \
	INT_TIME_STEP;
    if(temp < 0.0) {
	std::cout<<"State"<<std::endl;
	std::cout<<state[0]<<","<<state[1]<<","<<state[2]<<","<<state[3]<<","<<state[4]<<","<<state[5]<<std::endl;
	std::cout<<"Prev state"<<std::endl;
	std::cout<<prevState[0]<<","<<prevState[1]<<","<<prevState[2]<<","<<prevState[3]<<","<<prevState[4]<<","<<prevState[5]<<std::endl;
	std::cout<<"Control"<<std::endl;
	std::cout<<controlVec<<std::endl;
	std::cout<<temp<<std::endl;
    }
    return temp;
    // return \
    // 	(VELSCALE*abs((prevBodyVel.cross(bodyVel)/(prevBodyVel.norm()*bodyVel.norm()))(0,0)) + \
    // 	 ((controlVec - desControl).transpose()*Rf*(controlVec - desControl))(0,0)) * \
    // 	INT_TIME_STEP;
    
}
