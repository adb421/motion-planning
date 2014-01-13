#define STATE_SPACE_DIM 6
#define CONTROL_SPACE_DIM 3
#define DISC_S 2
#define DISC_FN 2
#define DISC_FT 2
#define NUM_DISC (DISC_S*DISC_FN*DISC_FT)
#define MU (1.0)
#define TIME_STEP (0.001)
#define INT_TIME_STEP (0.0001)
#define LO (0.051)
#define WO (0.045)
#define MO (0.07545)
#define JO (1.0/3.0*MO*(LO*LO + WO*WO))
#define GRAV (9.81*sin(0.405))
#define MAX_FN (5.0*GRAV*MO)
#define MIN_FN (0.0)
#define BACKWARDS_INT (0)
#define BIAS 0.01 //prob of sampling goal state exactly
#define TO_METERS 0.0254
#define MIN_X (-20*TO_METERS)
#define MAX_X (4*TO_METERS)
#define MIN_Y (-10*TO_METERS)
#define MAX_Y (30*TO_METERS)
#define MIN_TH (-3.0*M_PI)
#define MAX_TH (3.0*M_PI)
#define MAXVEL_XY (5)
#define MAXVEL_TH (5.0*M_PI)
#define GOAL_EPSILON (0.000001)
#define VELSCALE (0.0)
#define QC (100.0)//(1000.0)
#define S_STD_DEV (0.5*LO/5.0)
#define FN_STD_DEV (0.25*(MAX_FN-MIN_FN)/5.0)
#define MU_STD_DEV (0.5*MU/5.0)

