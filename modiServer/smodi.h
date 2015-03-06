#ifndef _MODI_H_
#define _MODI_H_

#define PI 3.14159265

//#define REAL_PAR      /* used when the real environments (or virtual small) are used */
#define SIM_PAR         /* used when the virtual environments are used */

/* geometry parameters */

#ifdef REAL_PAR
#define MODI_DIAMETER       0.078   /* diameter of the modi (7.8 [cm]) */
#endif
#ifdef SIM_PAR
#define MODI_DIAMETER       0.11   /* diameter of the modi (7.8 [cm]) */
#endif

#define YOFFSET             0       /* offset for the position of the proximity sensors */
#define ANGLE_BTW_PROXSENS  45.0    /* angle formed between the proximity sensors */

/* sensors parameters */

#define NCSENS              8       /* number of collision sensors */
#define MAX_OBJECTS         5       /* maximum amount of objects in the scene */
#define MAX_MODIES          5       /* maximum amount of modies in the scene */
#define DEFAULT_RANGE       1.0     /* default range for the proximity sensors (100[cm]) */
#define MIN_SENS_DIST       0.05    /* minimum sensing range for the prox. sensors (5[cm]) */
#define COLLISION_RADIUS    0.07    /* radius, from the center of the robot, of the collision sensors */
#define DEFAULT_SPEED       PI*0.5  /* default speed of each robot */


/// DETECTION RANGE MODIFIED!!
#ifdef SIM_PAR
#define DETECTION_RANGE     1.5     /* range for the object/modi detectors */
#endif
#ifdef REAL_PAR
#define DETECTION_RANGE     0.6     /* range for the object/modi detectors */
#endif

/* shared memory keys */

#define SH_SENSORS  50000                    /* keys for the sensors */
#define SH_MOTORS   SH_SENSORS+MAX_MODIES    /* keys for the motors */

/* alias for some of the options in the printUtil function */

#define MODI_DISTANCES  1           /* to print the distance between modies */
#define OBJ_DISTANCES   2           /* to print the distance towards objects */
#define COL_SENSORS     4           /* to print the boolean value of each collision sensor */

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "surf_wrapper.h"



typedef unsigned char uchar;

typedef struct{
    float left;
    float right;
} motor;

typedef struct{
    motor mot;
    float mo;
} motor_and_mo;

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

class smodi
{
public:
    smodi();
    pta get_pos();
    SDL_Rect getImgPos();
    float getMO();
    bool isEmpty();
    void printUtil(int what);
    void init(int idx,pta scale,SDL_Rect bgSize);
    void upd_data_from_vrep(float x, float y, float a);
    void upd_distances_to_modies(smodi *modi_arr, surf_wrapper *bg);
    void upd_distances_to_objects(mData *obj_arr, surf_wrapper *bg);
    void upd_mo_array(smodi *modi_arr);
    void draw_rays(surf_wrapper *bg);
    void upd_prox_sen(surf_wrapper* bg);
    void draw_prox_sen(surf_wrapper* bg);
    void upd_col_sen(surf_wrapper* bg);
    void draw_col_sen(surf_wrapper* bg);
    void upd_sensors_shm();
    void upd_motor_from_shm();
    motor get_data_to_vrep();
    void setMotors(float left, float right);
    void make_available();

private:
    void upd_rotpx();
    float check_visibility(float angle, float dist, surf_wrapper *bg);
    int sen_arr_size;
    pta *sen_arr;
    pta *sen_arr_l;
    pta *sen_arr_c;
    pta *sen_arr_r;
    pta scaleSize;
    SDL_Rect bgSize;
    SDL_Rect ImgPos;
    int dist_arr_size;
    pta col_sens[NCSENS*3];
    pta o_col_sens[NCSENS*3];
    int col_detect[NCSENS*3];
    int i,j,k,ii,jj,kk;
    motor  data_to_vrep;
    motor_and_mo *sh_data_to_vrep;
    ModiSen data_to_control;
    ModiSen *sh_data_to_control;
    int shmid,shcmdid;
    key_t key_sens,key_mot;

    // logger
    FILE *pfile;
};

#endif
