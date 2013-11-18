#define STATE_SPACE_DIM 6
#define CONTROL_SPACE_DIM 3
#define DISC_S 7
#define DISC_FN 9
#define DISC_FT 7
#define NUM_DISC (DISC_S*DISC_FN*DISC_FT)
#define MU (1.0)
#define TIME_STEP (0.01)
#define INT_TIME_STEP (0.0001)
#define LO (0.051)
#define WO (0.045)
#define MO (0.07545)
#define JO (1.0/3.0*MO*(LO*LO + WO*WO))
#define GRAV (9.81*sin(0.4))
#define MAX_FN (3.0*GRAV*MO)
#define MIN_FN (0.1)
#define BACKWARDS_INT 0
#define BIAS 0.075 //prob of sampling goal state exactly
#define TO_METERS 0.0254
#define MIN_X (-30*TO_METERS)
#define MAX_X (5*TO_METERS)
#define MIN_Y (-16*TO_METERS)
#define MAX_Y ( 34*TO_METERS)
#define MIN_TH (-2.0*M_PI)
#define MAX_TH (2.0*M_PI)
#define MAXVEL_XY (6)
#define MAXVEL_TH (4.0*M_PI)
#define GOAL_EPSILON (0.01)