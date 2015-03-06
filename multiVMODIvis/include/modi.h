#ifndef _MODI_H_
#define _MODI_H_
#include "bmap.h"
#include "comm_utils.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

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
    float m;
    float a;
} vec;

#ifndef USE_SENSORS
typedef struct mDataf
{
    int id;
    pta pos;
    uchar flag;
} mDataf;
#endif // USE_SENSORS
#ifdef USE_SENSORS
typedef struct mDataf
{
    pt pos;
    double angl;
    int senl;
    int senc;
    int senr;
    uchar speed;
    uchar flag;
} mDataff;
#endif

class modi
{
public:
    modi();
    modi(int offsx, int offsy, int range);
    modi(int inoffx, int inoffy, int inrange,int inpatt_id);
    #ifdef USE_SENSORS
    void updateSenPix();
    void updateSDist(bmap bitmap);
    void drawSensorsInto(unsigned char *dataPtr,int width, int height);
    #endif // USE_SENSORS
    void drawTrace(bmap bitmap,int radius);
    bool isVisible();
    void report(bool copy2file);
    void initLogger(char *filename);
    void initTraceLogger(int idx);
    void closeTraceLogger();
    void updTraceLogger();

    int getPattid();

    double getAngle();
    void setAngle(double angval);
    void updateAngle(double angval);

    int getKidx();
    void setKidx(int value);

    void setpos(int x, int y);
    void setpos(int inx, int iny,double incf);
    #ifndef USE_SENSORS
    pta getpos();
    #else
    pt getpos();
    #endif // USE_SENSORS
    mDataf* getmDatafref();

    void setshdata(int keydata,int keycmd);
    void updateshdata(bool comrdy);

    void setcmd(unsigned char *input);

    void setSocket(int i, modi* ref);
    int getSocket();
    void update_act(float ml, float mr, float id);
    void swait(){sem_wait(&sem);};
    void spost(){sem_post(&sem);};
    void sclose(){sem_close(&sem);};
private:
    // data of the robot (pos, angle, sensors values and speed
    mDataf data;
    #ifdef USE_SENSORS
    // virtual sensors
    int ofsx;
    int ofsy;
    pt *scen;
    pt *slef;
    pt *srig;
    int range;
    #endif // USE_SENSORS
    // fiducial detection
    int patt_id;
    int k_index;
    double cf;

    // shared object info
    int shmid,shcmdid;
    mDataf *sdata; unsigned char *scmd;
    key_t keydata,keycmd;

    // logger
    FILE *pfile;
    FILE *tracefile;

    void setval(unsigned char *dataPtr,int image_width, int image_height, int x, int y, unsigned char *value);

    // kalman filter values
    double th,ths,thp,thp0,P,Pp,fQR,Q,R,delta_thp,thpp,Kk;
    float outdata[3];

    char motl;
    char motr;

    // socket info
    int sockid;


    pthread_t tid;
    int thrd;
    sem_t   sem;
};

static int cm2px(int value)
{
    return (int) value*7.4;
}

static int px2cm(int value)
{
    return (int) ((float)value/7.4);
}

#endif
