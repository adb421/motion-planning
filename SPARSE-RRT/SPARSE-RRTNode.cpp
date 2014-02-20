#include "SPARSE-RRTNode.h"

double r = 1.0;

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

SPARSE_RRTNode::SPARSE_RRTNode() {
    parent = NULL;
    nodeTime = 0;
    for(int i = 0; i < STATE_SPACE_DIM; i++)
	nodeState(i,0) = 0;
//    nodeControl << 0, 1.0*GRAV*MO, 0;
    nodeControl << 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN;
}

SPARSE_RRTNode::SPARSE_RRTNode(StateVector_t setState) {
    parent = NULL;
    nodeTime = 0;
    nodeState = setState;
    nodeControl << 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN, 0.5*MAX_FN;
}

SPARSE_RRTNode::SPARSE_RRTNode(StateVector_t setState, ControlVector_t setControl, \
			       SPARSE_RRTNode* setParent, double setTime){
    parent = setParent;
    nodeTime = setTime;
    nodeState = setState;
    nodeControl = setControl;
}

ControlVector_t const & SPARSE_RRTNode::getNodeControl() const {
    return nodeControl;
}

StateVector_t const & SPARSE_RRTNode::getNodeState() const {
    return nodeState;
}

double SPARSE_RRTNode::getNodeTime() {
    return nodeTime;
}

SPARSE_RRTNode* SPARSE_RRTNode::getNodeParent(){
    return parent;
}

ControlVector_t				\
MapControlToWorld(StateVector_t state,	\
		  ControlVector_t controlArray) {
    ControlVector_t worldControl;
    // double conPoint = controlArray(0,0);
    // double Fn = controlArray(1,0);
    // double Ft = controlArray(2,0);
    double theta = state(4,0);
        double F1 = controlArray(0,0);
    double F2 = controlArray(1,0);
    double F3 = controlArray(2,0);
    double F4 = controlArray(3,0);
    worldControl(0,0) = (F4 + F2)*sin(BETA-theta) -  (F3 + F1)*sin(theta + BETA);
    worldControl(1,0) = (F1+F3)*cos(theta+BETA) + (F2 + F4)*cos(BETA-theta);
    worldControl(2,0) = LO*cos(BETA)*(F3+F4-F2-F1) + WO*sin(BETA)*(F2+F4-F1-F3);
    worldControl(3,0) = 0;
    // worldControl(0,0) = Ft*cos(theta) - Fn*sin(theta);
    // worldControl(1,0) = Fn*cos(theta) + Ft*sin(theta);
    // worldControl(2,0) = WO*Ft + (conPoint)*Fn;
    return worldControl;
}

StateVector_t OneStep(StateVector_t state, \
						 ControlVector_t worldControl) {
    StateVector_t derivState;
    derivState << state(1,0),
	worldControl(0,0)/MO,
	state(3,0),
	worldControl(1,0)/MO - GRAV,
	state(5,0),
	worldControl(2,0)/JO;
    state = state + derivState*INT_TIME_STEP;
    return state;
}

StateVector_t spawn(StateVector_t state, \
					       ControlVector_t control) {
    double time = 0;
    while(time < TIME_STEP) {
	state = OneStep(state, MapControlToWorld(state,control));
	time += INT_TIME_STEP;
    }
    return state;
}

StateVector_t RandomSample(StateVector_t goal) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0,1);
    if(dis(gen) < BIAS) {
	return goal;
    }
    //Not biased anymore
    StateVector_t samp;
    samp << (MIN_X + (MAX_X - MIN_X)*(dis(gen))),	\
	MAXVEL_XY*2*(dis(gen) - 0.5),			\
	(MIN_Y + (MAX_Y - MIN_Y)*(dis(gen))),		\
	MAXVEL_XY*2*(dis(gen) - 0.5),			\
	(MIN_TH + (MAX_TH - MIN_TH)*(dis(gen))),	\
	MAXVEL_TH*2*(dis(gen) - 0.5);
    return samp;
}

double dist(StateVector_t state1, StateVector_t state2) {
//    return (((state1 - state2).transpose())*QDist*(state1-state2))(0,0);
    double minTime = minTimePoly(getMinTimeCoefficients(state1, state2));
    double cost;
    if(minTime < 0)
	cost = HIGH_COST;
    else {
	double x01 = state1(0,0);
	double x02 = state1(1,0);
	double x03 = state1(2,0);
	double x04 = state1(3,0);
	double x05 = state1(4,0);
	double x06 = state1(5,0);
	double x11 = state2(0,0);
	double x12 = state2(1,0);
	double x13 = state2(2,0);
	double x14 = state2(3,0);
	double x15 = state2(4,0);
	double x16 = state2(5,0);
	double x5 = state1(4,0); //Theta linearized about
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
	std::cout<<", "<<minTime<<std::endl;
	std::cout<<"x0: "<<state1<<std::endl;
	std::cout<<"x1: "<<state2<<std::endl;
    }
    return cost;
}


ControlVector_t RandomControl(ControlVector_t controlMean, ControlVector_t controlGoal, std::array<int,CONTROL_SPACE_DIM> controlCare) {
    // for(int i = 0; i < controlCare.size(); i++) {
    // 	if(controlCare[i])
    // 	    controlMean(i,0) = (controlMean(i,0) + controlGoal(i,0))*0.5;
    // }

    std::random_device rd;
    std::mt19937 gen(rd());
    // std::normal_distribution<> SDis(controlMean(0,0), S_STD_DEV);
//    std::normal_distribution<> FnDis(controlMean(1,0), FN_STD_DEV);
    // std::normal_distribution<> MuDis(controlMean(2,0), MU_STD_DEV);
    std::uniform_real_distribution<double> FnDis(MIN_FN, MAX_FN);
//    std::normal_distribution<> FDis1(controlMean(0,0), FN_STD_DEV);
//    std::normal_distribution<> FDis2(controlMean(1,0), FN_STD_DEV);
//    std::normal_distribution<> FDis3(controlMean(2,0), FN_STD_DEV);
//    std::normal_distribution<> FDis4(controlMean(3,0), FN_STD_DEV);
    double FnSample = FnDis(gen);
//    std::normal_distribution<> FDisOther(FnSample, FN_STD_DEV);
    ControlVector_t samp;
    samp << FnDis(gen), FnDis(gen), FnDis(gen), FnDis(gen);
//    samp << FDis1(gen), FDis2(gen), FDis3(gen), FDis4(gen);
//    samp << FnSample, FDisOther(gen), FDisOther(gen), FDisOther(gen);
    for(int i = 0; i < CONTROL_SPACE_DIM; i++) {
	if(samp(i,0) > MAX_FN)
	    samp(i,0) = MAX_FN;
	else if(samp(i,0) < MIN_FN)
	    samp(i,0) = MIN_FN;
        }
    // double FnSamp = FnDis(gen);
    // if(FnSamp > MAX_FN)
    // 	FnSamp = MAX_FN;
    // else if(FnSamp < MIN_FN)
    // 	FnSamp = MIN_FN;
    // double muSamp = MuDis(gen);
    // if(muSamp > MU)
    // 	muSamp = MU;
    // else if(muSamp < -MU)
    // 	muSamp = -MU;
    // double conPointSamp = SDis(gen);

    //Check values
    // if(conPointSamp > LO)
    // 	conPointSamp = LO;
    // else if(conPointSamp < -LO)
    // 	conPointSamp = -LO;
    // samp << conPointSamp, FnSamp, muSamp*FnSamp;

    return samp;
}


std::array<int, STATE_SPACE_DIM> snapToGrid(SPARSE_RRTNode* nodeToSnap) {
    std::array<int, STATE_SPACE_DIM> gridValues;
    StateVector_t nodeState = nodeToSnap->getNodeState();
    std::array<double, STATE_SPACE_DIM> discVals = {GRID_DISC_X, GRID_DISC_VEL_XY, GRID_DISC_Y, GRID_DISC_VEL_XY, \
						 GRID_DISC_TH, GRID_DISC_VEL_TH};
    for(int i = 0; i < STATE_SPACE_DIM; i++) {
	gridValues[i] = (int) lround(nodeState(i,0)/discVals[i]);
    }
    return gridValues;
}

int violateConstraints(SPARSE_RRTNode* checkNode) {
    int violated = 0;
    StateVector_t state = \
	checkNode->getNodeState();
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
