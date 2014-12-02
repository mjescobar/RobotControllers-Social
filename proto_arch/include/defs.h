#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#define EPSIL 0.0000001

// a bigger number than the number of events
#define MAX_EVENTS  20

#define PI 3.14159265

/* geometry parameters */

#define MODI_DIAMETER       0.078   /* diameter of the modi (7.8 [cm]) */
#define YOFFSET             0       /* offset for the position of the proximity sensors */
#define ANGLE_BTW_PROXSENS  30.0    /* angle formed between the proximity sensors */

/* sensors parameters */

#define NCSENS              8       /* number of collision sensors */
#define MAX_OBJECTS         5       /* maximum amount of objects in the scene */
#define MAX_MODIES          5       /* maximum amount of modies in the scene */
#define DEFAULT_RANGE       1.0     /* default range for the proximity sensors (100[cm]) */
#define MIN_SENS_DIST       0.05    /* minimum sensing range for the prox. sensors (5[cm]) */
#define COLLISION_RADIUS    0.07    /* radius, from the center of the robot, of the collision sensors */
#define DEFAULT_SPEED       2*PI    /* default speed of each robot */
#define DETECTION_RANGE     1.0     /* range for the object/modi detectors */

#define NEAR_DIST   DEFAULT_RANGE                   /* a distance considered as "near", i.e, within the range of the proximity sensors */
#define FAR_DIST    DETECTION_RANGE                 /* distance to consider that a modi/object is far from something */
#define COL_DIST    COLLISION_RADIUS+MODI_DIAMETER  /* distance to consider that a modi/object is touching something */
#define MODI_FOLLOW_DIST 0.3                        /* optimal distance to follow a modi/wall */
#define WALL_FOLLOW_DIST 0.3

/* shared memory keys */

#define SH_SENSORS  50000                    /* keys for the sensors */
#define SH_MOTORS   SH_SENSORS+MAX_MODIES    /* keys for the motors */


// EVENTS
#define NULL_EVENT      0
#define COL_FRONT       1
#define COL_BACK        2
#define COL_LEFT        3
#define COL_RIGHT       4
#define NEAR_LEFT       5
#define NEAR_CENTER     6
#define NEAR_RIGHT      7
#define CORNER_LEFT     8
#define CORNER_RIGHT    9
#define OBJ_DETECT      10
#define MOD_DETECT      11
#define OBJ_TOUCH       12
#define MOD_TOUCH       13

// BEHAVIORS
#define MAX_BEHAVIORS   12

#define AP  1.0
#define AV  -1.0

#define NO_BEHAVIOR     -1
#define FULL_STOP       0
#define WALL_FOLLOW     1
#define REACH_OBJ       2
#define EVADE_OBJ       3
#define FREE_EXP        4
#define MARK_CORNERS    5
#define TURN_180        6 // flip
#define MODI_FOLLOW     7
#define MORE_SPD        8
#define LESS_SPD        9
#define EVADE_OBST      10

// motivational orientation defines
#define M_W1 0
#define M_W2 1
#define M_W3 2
#define M_W4 3
#define M_RO 1
#define M_BF 2
#define M_MO 0
#define M_EO 3

// behavior execution defines
#define MAX_SPD     1.5*DEFAULT_SPEED
#define KEEP_SPD    500
#define DEF_SPD     DEFAULT_SPEED
#define MIN_SPD     0.5*DEFAULT_SPEED   /* useful when you need a nonzero minimum speed */
#define DEF_DELTA   0.05*DEFAULT_SPEED

#define ANGLE_ERROR 6
#define CW 1
#define CCW 0

#define ANY_OBJ     -2
#define ANY_MODI    -2
#define NO_MODI     -1


#ifndef DEFS_H_INCLUDED
#define DEFS_H_INCLUDED

// 2D point
typedef struct{
    int x;
    int y;
} pt;
typedef unsigned char uchar;

typedef struct{
    float left;
    float right;
} motor;

typedef struct{
    motor mot;
    float mo;
} motor_and_mo;

typedef struct{
    float x;
    float y;
    float a;
} pta;

typedef struct{
    float m;
    float a;
} vec;

// struct with all the sensors
typedef struct{
    int id;                     /* id of this modi */
    float senl;                 /* proximity sensor left   */
    float senc;                 /* proximity sensor center */
    float senr;                 /* proximity sensor right  */
    int colsen[NCSENS];         /* eight collision detectors, one every 45 deg */
    vec m_pos[MAX_MODIES];      /* distance and angle with visible modies. not visible are -1 */
    vec o_pos[MAX_OBJECTS];     /* distance and angle with visible objects. not visible are -1 */
    float mo_sur[MAX_MODIES];   /* motivational orientation of visible modies */
    pta pos;                    /* position and angle */
    float mo;                   /* motivational orientation of THIS modi */
} ModiSen;

typedef struct mData
{
    int id;
    pta pos;
} mData;


float absv(float x);

#endif // DEFS_H_INCLUDED
