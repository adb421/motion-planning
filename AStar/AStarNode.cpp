#include "AStarNode.h"

double Qf_array[STATE_SPACE_DIM*STATE_SPACE_DIM] = {1000.0/pow((MAX_X - MIN_X),2),0,0,1000.0/pow((MAXVEL_XY*2),2)};

double r = 0.1;

double R_array[CONTROL_SPACE_DIM*CONTROL_SPACE_DIM] =		\
{r};


Eigen::Matrix<double, STATE_SPACE_DIM, STATE_SPACE_DIM> Qf = Eigen::Map<Eigen::MatrixXd>(Qf_array, STATE_SPACE_DIM, STATE_SPACE_DIM);
Eigen::Matrix<double, CONTROL_SPACE_DIM, CONTROL_SPACE_DIM> Rf = Eigen::Map<Eigen::MatrixXd>(R_array, CONTROL_SPACE_DIM, CONTROL_SPACE_DIM);

//Default Constructor
//Assume root of tree with zero initial state
AStarNode::AStarNode() {
    parent = NULL;
    good = true;
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
AStarNode::AStarNode(StateVector_t setState, StateVector_t setGoal) {
    parent = NULL;
    good = true;
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
AStarNode::AStarNode(AStarNode *setParent, StateVector_t setState, \
		     ControlVector_t setControl, \
		     StateVector_t setGoal, \
		     double setCost, double setTime) {
    good = true;
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
StateVector_t const & AStarNode::getNodeState() const
{
    return nodeState;
}

//Return the control used to get to the node
ControlVector_t const & AStarNode::getNodeControl() const
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
std::vector<AStarNode*> AStarNode::expand(map_t &grid)
{
    Eigen::ArrayXd F1 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    ControlVector_t controlArray;
    std::vector<AStarNode*> childNodes;//(DISC_S*DISC_FN*DISC_FT);
    std::vector<AStarNode*> tempChildren;
    AStarNode* tempNode;

    int i;
    for(i =  0; i < F1.size(); i++)
    {
	controlArray << F1(i,0);
	tempNode = this->spawn(controlArray);
	if(!violateConstraints(tempNode)) {
	    if(grid.count(snapToGrid(tempNode))==0) {
		tempChildren.push_back(tempNode);
		//Put it on the embedded grid
		grid.insert(make_pair(snapToGrid(tempNode),tempNode));
	    }
	    // else if(grid[snapToGrid(tempNode)]->getNodePriority() > tempNode->getNodePriority()){
	    //     tempChildren.push_back(tempNode);
	    //     grid[snapToGrid(tempNode)]->good = false;
	    //     grid.erase(grid.find(snapToGrid(tempNode)));
	    //     grid[snapToGrid(tempNode)] = tempNode;

	    // } else {
	    else {
		delete tempNode;
	    }
	}
	else {
	    delete tempNode;
	}
    }
    for(int it = 0; it < tempChildren.size(); it++) {
	childNodes.push_back(tempChildren.back());
	tempChildren.pop_back();
    }
    return childNodes;
}

int violateConstraints(AStarNode* checkNode) {
    int violated = 0;
    StateVector_t state = \
	checkNode->getNodeState();
    if(state(0,0) > MAX_X || state(0,0) < MIN_X) {
	violated = 1;
    } else if(fabs(state(1,0)) > MAXVEL_XY) {
	violated = 1;
    }
    return violated;
}

AStarNode* AStarNode::spawn(ControlVector_t controlArray) {
    StateVector_t state = nodeState;
    StateVector_t prevState;
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
    AStarNode* tmp = new AStarNode(this, state, controlArray, goalState, newCost, time);
    return tmp;
}

//Contact, normal, tangent - control
//xo, xod, yo, yod, tho, thod - state
ControlVector_t MapControlToWorld(StateVector_t state, ControlVector_t controlArray) {
    ControlVector_t worldControl;
    double F1 = controlArray(0,0);
    worldControl(0,0) = F1;
    return worldControl;
}

double costToGo(StateVector_t state, StateVector_t goalState)
{
    return 0.0;
    // StateVector_t errorVec = state - goalState;


    // double stateCost = (errorVec.transpose()*Qf*errorVec)(0,0);
    // return stateCost;
}

StateVector_t OneStep(StateVector_t state, ControlVector_t worldControl) {
    StateVector_t derivState;
    derivState << state(1,0), worldControl(0,0)/MO - GRAV;
    if(!BACKWARDS_INT)
	state += derivState*INT_TIME_STEP;
    else
	state -= derivState*INT_TIME_STEP;
    return state;
}

double realCost(StateVector_t state, ControlVector_t controlArray, StateVector_t prevState, StateVector_t goalState) {
    
    double controlCost = \
	(controlArray.transpose()*Rf*controlArray)(0,0);

    double stateCost =							\
    	((state - goalState).transpose()*Qf*(state - goalState))(0,0);
//    return (controlCost + 1 + stateCost)*INT_TIME_STEP;
//    return (controlCost + 1)*INT_TIME_STEP;
    return (controlCost + stateCost)*INT_TIME_STEP;
}

double euclideanDistance(StateVector_t state1, StateVector_t state2) {
    return (((state1 - state2).transpose())*(state1-state2))(0,0);
}

std::array<int, STATE_SPACE_DIM> snapToGrid(AStarNode* nodeToSnap) {
    std::array<int, STATE_SPACE_DIM> gridValues;
    StateVector_t nodeState = nodeToSnap->getNodeState();
    std::array<double, STATE_SPACE_DIM> discVals = {GRID_DISC_Y, GRID_DISC_VEL_XY};
    for(int i = 0; i < STATE_SPACE_DIM; i++) {
	gridValues[i] = (int) lround(nodeState(i,0)/discVals[i]);
    }

    return gridValues;
}
