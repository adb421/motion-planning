#include "AStarNode.h"

double Qf_array[STATE_SPACE_DIM*STATE_SPACE_DIM] =			\
{1/pow((MAX_X - MIN_X),2), 0,   0, 0, 0, 0,				\
 0,      1/pow((MAXVEL_XY*2),2), 0, 0, 0, 0,				\
 0,       0,   1/pow((MAX_Y - MIN_Y),2), 0, 0, 0,			\
 0,       0,   0,      1/pow((MAXVEL_XY*2),2), 0, 0,			\
 0,       0,   0,      0,   1/pow((MAX_TH - MIN_TH),2), 0,		\
 0,       0,   0,      0,   0,      1/pow((MAXVEL_TH*2),2)};

// double R_array[9] = {1.0/pow(LO*2.0,2), 0,   0,	\
// 		     0,   1.0/pow(MAX_FN-MIN_FN,2), 0,	\
// 		     0,   0,   1.0/pow(MU*2.0,2)};
double R_array[16] = {1.0/pow(MAX_FN-MIN_FN,2), 0, 0, 0,	\
		      0, 1.0/pow(MAX_FN-MIN_FN,2), 0, 0,	\
		      0, 0, 1.0/pow(MAX_FN-MIN_FN,2), 0,	\
		      0, 0, ,0, 1.0/pow(MAX_FN-MIN_FN,2)};


Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> Qf = Eigen::Map<Eigen::MatrixXd>(Qf_array, 6, 6);
Eigen::Matrix<double, CONTROL_SPACE_DIM, CONTROL_SPACE_DIM> Rf = Eigen::Map<Eigen::MatrixXd>(R_array, 3, 3);

//Default Constructor
//Assume root of tree with zero initial state
AStarNode::AStarNode() {
    parent = NULL;
    int j;
    for(j = 0; j < STATE_SPACE_DIM; j++)
    {
	nodeState(j,0) = 0.0;
	goalState(j,0) = 0.0;
    }
    for(j = 0; j < CONTROL_SPACE_DIM; j++)
	nodeControl(j,0) = 0.0;
    cost = 0.0; heur = 0.0; nodeTime = 0.0;
}

//Constructor for root
AStarNode::AStarNode(Eigen::Matrix<double, STATE_SPACE_DIM,1> setState, Eigen::Matrix<double, STATE_SPACE_DIM, 1> setGoal) {
    parent = NULL;
    nodeState = setState;
    goalState = setGoal;
    int j;
    for(j = 0; j < CONTROL_SPACE_DIM; j++)
	nodeControl(j,0) = 0.0;
    cost = 0.0;
    nodeTime = 0.0;
    heur = costToGo(nodeState, goalState);
}

//Constructor for most nodes
AStarNode::AStarNode(AStarNode *setParent, Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> setState, \
		     Eigen::Matrix<double, CONTROL_SPACE_DIM,1> setControl, \
		     Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> setGoal, \
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
Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> const & AStarNode::getNodeState() const
{
    return nodeState;
}

//Return the control used to get to the node
Eigen::Matrix<double, CONTROL_SPACE_DIM,1> const & AStarNode::getNodeControl() const
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
    Eigen::ArrayXd F1 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F2 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F3 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F4 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray;
    std::vector<AStarNode*> childNodes;//(DISC_S*DISC_FN*DISC_FT);
    std::vector<AStarNode*> tempChildren;
    AStarNode* tempNode;

    int i, j, k, l;
    for(i =  0; i < F1.size(); i++)
    {
	for(j = 0; j < F2.size(); j++)
	{
	    for(k = 0; k < F3.size(); k++)
	    {
		for(l = 0; l < F4.size(); l++) {
		    controlArray << F1(i,0), F2(j,0), F3(k,0), F4(l,0);
		    tempNode = this->spawn(controlArray);
		    tempChildren.push_back(tempNode);
		}
		for(int it = 0; it < tempChildren.size(); it++) {
		    childNodes.push_back(tempChildren.back());
		    tempChildren.pop_back();
		}
	    }
	}
    }
    return childNodes;
}

AStarNode* AStarNode::spawn(Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray) {
    Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state = nodeState;
    Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> prevState;
    double time = nodeTime;
    double newCost = cost;
    while(time <= TIME_STEP + nodeTime) {
	prevState = state;
	//Euler integrate
	state = OneStep(state, MapControlToWorld(state, controlArray));
	//Increment cost
	newCost += realCost(state, controlArray, prevState, goalState);
//	newCost += realCost(state, controlArray, goalState);
	time += INT_TIME_STEP;
    }
    return new AStarNode(this, state, controlArray, goalState, newCost, time);
}

//Contact, normal, tangent - control
//xo, xod, yo, yod, tho, thod - state
Eigen::Matrix<double, CONTROL_SPACE_DIM,1> MapControlToWorld(Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray) {
    Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl;
    double theta = state[4];
    double F1 = controlArray[0];
    double F2 = controlArray[1];
    double F3 = controlArray[2];
    double F4 = controlArray[3];
    worldControl(0,0) = (F4 + F2)*sin(BETA-theta) -  (F3 + F1)*sin(theta + BETA);
    worldControl(1,0) = (F1+F3)*cos(theta+BETA) + (F2 + F4)*cos(BETA-theta);
    worldControl(2,0) = LO*cos(BETA)*(F3+F4-F2-F1) + WO*sin(BETA)*(F2+F4-F1-F3);
    return worldControl;
}

double costToGo(Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state, Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> goalState)
{
    Eigen::Matrix<double, STATE_SPACE_DIM, 1> errorVec = stateVec - goalVec;


    double stateCost = (errorVec.transpose()*Qf*errorVec)(0,0);
    return stateCost;
}

Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> OneStep(Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> worldControl) {
    Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> derivState = {state[1], worldControl[0]/MO, \
									  state[3], worldControl[1]/MO - GRAV, \
									  state[5], worldControl[2]/JO};
    if(!BACKWARDS_INT)
	state += derivState*INT_TIME_STEP;
    else
	state -= derivState*INT_TIME_STEP;
    return state;
}

double realCost(Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> state, Eigen::Matrix<double, CONTROL_SPACE_DIM,1> controlArray, Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> prevState, Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> goalState) {

    desControl << -LO, GRAV*MO, 0;
    
    double controlCost = \
	((controlVec - desControl).transpose()*Rf*(controlVec - desControl))(0,0);

    double stateCost =						\
	((stateVec - goalVec).transpose()*Qf*(stateVec - goalVec))(0,0);
    return (controlCost + stateCost)*INT_TIME_STEP;    
}
