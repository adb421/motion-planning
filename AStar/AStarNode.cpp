#include "AStarNode.h"
int badCount = 0;
void printBadCount() {
    std::cout<<badCount<<std::endl;
}
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
//double r = 1000000.0;
double r = 1;
// double r = 0.0000001;
//double r = 0.000000001;
double R_array[CONTROL_SPACE_DIM*CONTROL_SPACE_DIM] =		\
{r, 0, 0, 0,				\
 0, r, 0, 0,				\
 0, 0, r, 0,				\
 0, 0, 0, r};


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
AStarNode::AStarNode(AStarNode_ptr setParent, StateVector_t setState, \
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
AStarNode_ptr AStarNode::getNodeParent()
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
std::vector<AStarNode_ptr> AStarNode::expand(map_t &grid)
{
    Eigen::ArrayXd F1 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F2 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F3 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    Eigen::ArrayXd F4 = Eigen::ArrayXd::LinSpaced(DISC_FN,MIN_FN,MAX_FN);
    ControlVector_t controlArray;
    std::vector<AStarNode_ptr> childNodes;//(DISC_S*DISC_FN*DISC_FT);
    std::vector<AStarNode_ptr> tempChildren;
    AStarNode_ptr tempNode;

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
		    if(!violateConstraints(tempNode)) {
			if(grid.count(snapToGrid(tempNode))==0) {
			    tempChildren.push_back(tempNode);
			    //Put it on the embedded grid
			    grid.insert(pair_t(snapToGrid(tempNode),tempNode));
//			    grid[snapToGrid(tempNode)] = tempNode;
			    std::array<int, STATE_SPACE_DIM> tmp;
			    tmp = snapToGrid(tempNode);
			    std::cout<<"S:"<<std::endl;
			    for(int ii = 0; ii < STATE_SPACE_DIM; ii++) 
				std::cout<<tmp[ii]<<std::endl;
			    std::cout<<"Count: "<<grid.count(tmp)<<std::endl;
			    std::cout<<"Done."<<std::endl;
			}
			else if(grid[snapToGrid(tempNode)].ptr->getNodePriority() > tempNode.ptr->getNodePriority()){
			    tempChildren.push_back(tempNode);
			    grid[snapToGrid(tempNode)].ptr->good = false;
			    grid.erase(grid.find(snapToGrid(tempNode)));
			    grid[snapToGrid(tempNode)] = tempNode;

			} else {
//			else {
//			    badCount++;
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
	    }
	}
    }
    // std::cout<<"Node cost to go: "<<heur<<std::endl;
    // for(int it = 0; it < childNodes.size(); it++) {
    // 	std::cout<<"Child #"<<it<<" cost to go: "<< childNodes[it]->getNodeCostToGo() << std::endl;
    // }
    // int tmp;
    // std::cin>>tmp;
    return childNodes;
}

int violateConstraints(AStarNode_ptr checkNode) {
    int violated = 0;
    StateVector_t state = \
	checkNode.ptr->getNodeState();
    if(state(0,0) > MAX_X || state(0,0) < MIN_X) {
	violated = 1;
    } else if(fabs(state(1,0)) > MAXVEL_XY) {
	violated = 1;
    } else if(state(2,0) > MAX_Y || state(2,0) < MIN_Y) {
	violated = 1;
    } else if(fabs(state(3,0)) > MAXVEL_XY) {
	violated = 1;
    } else if(state(4,0) > MAX_TH || state(4,0) < MIN_TH) {
	violated = 1;
    } else if(fabs(state(5,0)) > MAXVEL_TH) {
	violated = 1;
    }
    return violated;
}

AStarNode_ptr AStarNode::spawn(ControlVector_t controlArray) {
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
    AStarNode_ptr tmp;
    tmp.ptr = new AStarNode(this, state, controlArray, goalState, newCost, time);
    return tmp;
}

//Contact, normal, tangent - control
//xo, xod, yo, yod, tho, thod - state
ControlVector_t MapControlToWorld(StateVector_t state, ControlVector_t controlArray) {
    ControlVector_t worldControl;
    double theta = state(4,0);
    double F1 = controlArray(0,0);
    double F2 = controlArray(1,0);
    double F3 = controlArray(2,0);
    double F4 = controlArray(3,0);
    worldControl(0,0) = (F4 + F2)*sin(BETA-theta) -  (F3 + F1)*sin(theta + BETA);
    worldControl(1,0) = (F1+F3)*cos(theta+BETA) + (F2 + F4)*cos(BETA-theta);
    worldControl(2,0) = LO*cos(BETA)*(F3+F4-F2-F1) + WO*sin(BETA)*(F2+F4-F1-F3);
    worldControl(3,0) = 0;
    return worldControl;
}

double costToGo(StateVector_t state, StateVector_t goalState)
{
    // StateVector_t errorVec = state - goalState;


    // double stateCost = (errorVec.transpose()*Qf*errorVec)(0,0);
    // return stateCost;
    double minTime = minTimePoly(getMinTimeCoefficients(state, goalState));
    double cost;
    if(minTime < 0)
	cost = HIGH_COST;
    else {
	double x01 = state(0,0);
	double x02 = state(1,0);
	double x03 = state(2,0);
	double x04 = state(3,0);
	double x05 = state(4,0);
	double x06 = state(5,0);
	double x11 = goalState(0,0);
	double x12 = goalState(1,0);
	double x13 = goalState(2,0);
	double x14 = goalState(3,0);
	double x15 = goalState(4,0);
	double x16 = goalState(5,0);
	double x5 = state(4,0); //Theta linearized about
	cost = (4.0*pow(minTime,4) + (r*(pow(MO,2)*(pow(GRAV,2)*pow(minTime,4) + 12.0*(-pow(x01,2) + pow(x03,2)) + \
						    2.0*GRAV*pow(minTime,3)*(-x04 + x14) + 4.0*(3.0*(2.0*x01*x11 - pow(x11,2) - 2.0*x03*x13 + pow(x13,2)) + \
												pow(minTime,2)*(-pow(x02,2) + pow(x04,2) - x02*x12 - pow(x12,2) + x04*x14 + pow(x14,2)) - \
												3.0*minTime*((x01 - x11)*(x02 + x12) - (x03 - x13)*(x04 + x14))))*(-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))* \
					 cos(2.0*x5)*pow(1.0/sin(2.0*BETA),2) - 4.0*JO*MO*WO*(6.0*x11*(-x05 + x15) + pow(minTime,2)*(2.0*x02*x06 + x06*x12 + x02*x16 + 2.0*x12*x16) + \
											      3.0*x01*(2.0*x05 - 2.0*x15 + minTime*(x06 + x16)) + 3.0*minTime*((x02 + x12)*(x05 - x15) - x11*(x06 + x16)))*cos(x5)*pow(1.0/cos(BETA),2) \
					 + (pow(LO,2)*pow(MO,2)*(pow(GRAV,2)*pow(minTime,4) + 12.0*(pow(x01,2) + pow(x03,2)) + \
								 2.0*GRAV*pow(minTime,3)*(-x04 + x14) + 4.0* \
								 (3.0*(-2.0*x01*x11 + pow(x11,2) - 2.0*x03*x13 + pow(x13,2)) + \
								  pow(minTime,2)*(pow(x02,2) + pow(x04,2) + x02*x12 + pow(x12,2) + x04*x14 + pow(x14,2)) + \
								  3.0*minTime*((x01 - x11)*(x02 + x12) + (x03 - x13)*(x04 + x14))))*pow(1.0/sin(BETA),2) + \
					    (pow(GRAV,2)*pow(minTime,4)*pow(MO,2)*(pow(LO,2) + pow(WO,2)) - \
					     2.0*GRAV*pow(minTime,3)*pow(MO,2)*(pow(LO,2) + pow(WO,2))*(x04 - x14) + \
					     4.0*(6.0*pow(JO,2)*pow(x05,2) + pow(LO,2)*pow(MO,2)* \
						  (3.0*pow(x01,2) + 3.0*x01*(-2.0*x11 + minTime*(x02 + x12)) + 3.0*(pow(x11,2) + pow(x03 - x13,2)) + \
						   pow(minTime,2)*(pow(x02,2) + pow(x04,2) + x02*x12 + pow(x12,2) + x04*x14 + pow(x14,2)) - \
						   3.0*minTime*(x11*(x02 + x12) - (x03 - x13)*(x04 + x14))) + \
						  pow(MO,2)*pow(WO,2)*(3*pow(x01,2) + 3.0*x01*(-2.0*x11 + minTime*(x02 + x12)) + 3.0*(pow(x11,2) + pow(x03 - x13,2)) + \
								       pow(minTime,2)*(pow(x02,2) + pow(x04,2) + x02*x12 + pow(x12,2) + x04*x14 + pow(x14,2)) - \
								       3.0*minTime*(x11*(x02 + x12) - (x03 - x13)*(x04 + x14))) + \
						  2.0*pow(JO,2)*(3.0*x15*(-2.0*x05 + x15) + 3.0*minTime*(x05 - x15)*(x06 + x16) + \
								 pow(minTime,2)*(pow(x06,2) + x06*x16 + pow(x16,2)))))*pow(1.0/cos(BETA),2))/2.0 - 
					 2.0*JO*MO*WO*(12.0*x13*(-x05 + x15) + GRAV*pow(minTime,3)*(-x06 + x16) + \
						       2.0*pow(minTime,2)*(2.0*x04*x06 + x06*x14 + x04*x16 + 2.0*x14*x16) + 6.0*x03*(2.0*x05 - 2.0*x15 + minTime*(x06 + x16)) + \
						       6.0*minTime*((x04 + x14)*(x05 - x15) - x13*(x06 + x16)))*pow(1.0/cos(BETA),2)*sin(x5) + \
					 2.0*pow(MO,2)*(-12.0*x01*x03 + GRAV*pow(minTime,3)*(x02 - x12) + 12.0*(x03*x11 + (x01 - x11)*x13) - \
							6.0*minTime*((x02 + x12)*(x03 - x13) + (x01 - x11)*(x04 + x14)) - 2.0*pow(minTime,2)*(x02*(2.0*x04 + x14) + x12*(x04 + 2.0*x14)))* \
					 (-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))*pow(1.0/sin(2.0*BETA),2)*sin(2.0*x5)))/pow(LO,2))/(4.0*pow(minTime,3));
    }
    if(cost < 0.0) {
	std::cout<<cost<<", "<<minTime<<std::endl;
	std::cout<<"x0: "<<state<<std::endl;
	std::cout<<"x1: "<<goalState<<std::endl;
    }
    return cost;
}

StateVector_t OneStep(StateVector_t state, ControlVector_t worldControl) {
    StateVector_t derivState;
    derivState << state(1,0), worldControl(0,0)/MO,			\
	state(2,0), worldControl(1,0)/MO - GRAV, \
	state(5,0), worldControl(2,0)/JO;
    if(!BACKWARDS_INT)
	state += derivState*INT_TIME_STEP;
    else
	state -= derivState*INT_TIME_STEP;
    return state;
}

double realCost(StateVector_t state, ControlVector_t controlArray, StateVector_t prevState, StateVector_t goalState) {
    
    double controlCost = \
	(controlArray.transpose()*Rf*controlArray)(0,0);

    // double stateCost =						\
    // 	((state - goalState).transpose()*Qf*(state - goalState))(0,0);
    // return (controlCost + 1 + stateCost)*INT_TIME_STEP;
    return (controlCost + 1)*INT_TIME_STEP;
}

//Coeffs go a + b t + c t^2 + d t^3 + e t^4 , [a, b, c, d, e]
Eigen::VectorXcd polynomialRoots(std::array<double,5> coeffs) {
    double c0 = coeffs[0]/coeffs[4];
    double c1 = coeffs[1]/coeffs[4];
    double c2 = coeffs[2]/coeffs[4];
    double c3 = coeffs[3]/coeffs[4];
    Eigen::Matrix<double,4,4> CompMatrix;
    CompMatrix << 0, 0, 0, -c0, \
	1, 0, 0, -c1, \
	0, 1, 0, -c2, \
	0, 0, 1, -c3;

    Eigen::EigenSolver<Eigen::Matrix<double,4,4>> es(CompMatrix);
//    std::cout<<es.eigenvalues()<<std::endl;
    return es.eigenvalues();
}

double minTimePoly(std::array<double,5> coeffs) {

    Eigen::VectorXcd eigs = polynomialRoots(coeffs);
    std::vector<double> timesToCheck;

    for(int i = 0; i < eigs.rows(); i++) {
	if(fabs((eigs(i,0)).imag()) < 0.000001)
	    timesToCheck.push_back(eigs(i,0).real());
    }
    double minTime = -1;
    for(int j = 0; j < timesToCheck.size(); j++) {
	if(timesToCheck[j] > 0) {
	    if(minTime < 0)
		minTime = timesToCheck[j];
	    else if(timesToCheck[j] < minTime)
		minTime = timesToCheck[j];
	}
    }

    return minTime;
}

std::array<double,5> getMinTimeCoefficients(StateVector_t x0, StateVector_t x1) {

    std::array<double,5> coeffs;
    double x01 = x0(0,0);
    double x02 = x0(1,0);
    double x03 = x0(2,0);
    double x04 = x0(3,0);
    double x05 = x0(4,0);
    double x06 = x0(5,0);
    double x11 = x1(0,0);
    double x12 = x1(1,0);
    double x13 = x1(2,0);
    double x14 = x1(3,0);
    double x15 = x1(4,0);
    double x16 = x1(5,0);
    double x5 = x0(4,0); //Theta linearized about

    coeffs[4] =((2.0*pow(LO,2)*(1.0 + pow(GRAV,2)*pow(MO,2)*r) + \
		 pow(GRAV,2)*pow(MO,2)*r*pow(WO,2) - \
		 2.0*pow(LO,2)*cos(4.0*BETA) + \
		 pow(GRAV,2)*pow(MO,2)*r* \
		 (-(pow(WO,2)*cos(2.0*x5)) + \
		  cos(2.0*BETA)*(-pow(WO,2) + \
				 (-2.0*pow(LO,2) + pow(WO,2))*cos(2*x5))))* \
		pow(1.0/sin(2.0*BETA),2))/(4.*pow(LO,2));

    coeffs[3] = 0.0;

    coeffs[2] = (r*(pow(MO,2)*(pow(x02,2) - pow(x04,2) + x02*x12 + pow(x12,2) - \
			       x04*x14 - pow(x14,2))* \
		    (-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))* \
		    cos(2.0*x5)*pow(1.0/sin(2.0*BETA),2) + \
		    JO*MO*WO*(x02*(2*x06 + x16) + x12*(x06 + 2.0*x16))*cos(x5)* \
		    pow(1.0/cos(BETA),2) + (-(pow(LO,2)*pow(MO,2)* \
					      (pow(x02,2) + pow(x04,2) + x02*x12 + pow(x12,2) + \
					       x04*x14 + pow(x14,2))*pow(1.0/sin(BETA),2)) - \
					    (pow(MO,2)*(pow(LO,2) + pow(WO,2))* \
					     (pow(x02,2) + pow(x04,2) + x02*x12 + pow(x12,2) + \
					      x04*x14 + pow(x14,2)) + \
					     2.0*pow(JO,2)*(pow(x06,2) + x06*x16 + pow(x16,2)))* \
					    pow(1.0/cos(BETA),2))/2.0 + \
		    JO*MO*WO*(x04*(2*x06 + x16) + x14*(x06 + 2.0*x16))*pow(1.0/cos(BETA),2)* \
		    sin(x5) + pow(MO,2)*(x02*(2.0*x04 + x14) + x12*(x04 + 2.0*x14))* \
		    (-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))* \
		    pow(1.0/sin(2.0*BETA),2)*sin(2.0*x5)))/pow(LO,2);
    
    coeffs[1] = (6.0*r*(pow(MO,2)*((x01 - x11)*(x02 + x12) - (x03 - x13)*(x04 + x14))* \
			(-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))* \
			cos(2.0*x5)*pow(1.0/sin(2.0*BETA),2) + \
			JO*MO*WO*((x02 + x12)*(x05 - x15) + (x01 - x11)*(x06 + x16))* \
			cos(x5)*pow(1.0/cos(BETA),2) + \
			(pow(LO,2)*pow(MO,2)* \
			 (-(x01*(x02 + x12)) + x11*(x02 + x12) - \
			  (x03 - x13)*(x04 + x14))*pow(1.0/sin(BETA),2) + \
			 (-(pow(MO,2)*(pow(LO,2) + pow(WO,2))* \
			    ((x01 - x11)*(x02 + x12) + (x03 - x13)*(x04 + x14))) - \
			  2.0*pow(JO,2)*(x05 - x15)*(x06 + x16))*pow(1.0/cos(BETA),2))/2.0 \
			+ JO*MO*WO*((x04 + x14)*(x05 - x15) + (x03 - x13)*(x06 + x16))*\
			pow(1.0/cos(BETA),2)*sin(x5) + \
			pow(MO,2)*((x02 + x12)*(x03 - x13) + (x01 - x11)*(x04 + x14))*\
			(-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))*\
			pow(1.0/sin(2.0*BETA),2)*sin(2.0*x5)))/pow(LO,2);

    coeffs[0] = (9.0*r*(pow(MO,2)*(x01 + x03 - x11 - x13)*(x01 - x03 - x11 + x13)*\
			(-pow(WO,2) + (-2.0*pow(LO,2) + pow(WO,2))*cos(2.0*BETA))*\
			cos(2.0*x5)*pow(1.0/sin(2.0*BETA),2) + \
			2.0*JO*MO*WO*(x01 - x11)*(x05 - x15)*cos(x5)*pow(1.0/cos(BETA),2) + \
			(-(pow(LO,2)*pow(MO,2)*\
			   (pow(x01 - x11,2) + pow(x03 - x13,2))*pow(1.0/sin(BETA),2))\
			 - (pow(MO,2)*(pow(LO,2) + pow(WO,2))*\
			    (pow(x01 - x11,2) + pow(x03 - x13,2)) + \
			    2.0*pow(JO,2)*pow(x05 - x15,2))*pow(1.0/cos(BETA),2))/2.0 + \
			2.0*JO*MO*WO*(x03 - x13)*(x05 - x15)*pow(1.0/cos(BETA),2)*sin(x5) - \
			2.0*pow(MO,2)*(x01 - x11)*(x03 - x13)*\
			(pow(WO,2) + (2.0*pow(LO,2) - pow(WO,2))*cos(2.0*BETA))*\
			pow(1.0/sin(2.0*BETA),2)*sin(2.0*x5)))/pow(LO,2);

    return coeffs;
}

double euclideanDistance(StateVector_t state1, StateVector_t state2) {
    return (((state1 - state2).transpose())*(state1-state2))(0,0);
}

std::array<int, STATE_SPACE_DIM> snapToGrid(AStarNode_ptr nodeToSnap) {
    std::array<int, STATE_SPACE_DIM> gridValues;
    StateVector_t nodeState = nodeToSnap.ptr->getNodeState();
    std::array<double, STATE_SPACE_DIM> discVals = {GRID_DISC_X, GRID_DISC_VEL_XY, GRID_DISC_Y, GRID_DISC_VEL_XY, \
						 GRID_DISC_TH, GRID_DISC_VEL_TH};
    for(int i = 0; i < STATE_SPACE_DIM; i++) {
	gridValues[i] = (int) lround(nodeState(i,0)/discVals[i]);
    }

    return gridValues;
}
