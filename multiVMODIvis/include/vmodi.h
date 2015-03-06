#ifndef _VMODI_H_
#define _VMODI_H_

#include <stdio.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glut.h>
#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <time.h>

#include <sys/types.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <xbee.h>
#include <string.h>
//#include "modi.h"
#include "object.h"

#include <pthread.h>

#define MAXMODIES 10
#define MAXOBJECTS 10
#define FRAMES_PER_COM 2

typedef struct xbee_conAddress xbeeAddr;
typedef struct xbee_con xbeecon;
// VARIABLE DECLARATIONS

//
// Camera configuration.
//
char *vconf = (char *) "";

int xsize, ysize;
int thresh = 100;
int count = 0;

char *cparam_name    = (char *) "Data/camera_para.dat";
ARParam cparam;

char *patt_name1      = (char *) "Data/patt.cubito";
int patt_id1;


double patt_width     = 100.0;
double patt_center[2] = {0.0, 0.0};


double patt_trans2[3][4];

int image_width = 1920;
int image_height = 1080;
unsigned char *pframe;


char scenario[200];
char modishape[200];
char objshape[200];

// artoolkit functions

static void   init(char * conf1Name, char * conf2Name, char *outFile);
static void   cleanup(void);
static void   keyEvent( unsigned char key, int x, int y);
static void   mainLoop(void);

int toggle_reg = 0;

// modis

modi modies[MAXMODIES];
std::string patterns[MAXMODIES];
int shkeys[MAXMODIES*2];
//  number of modies created
int modinum = 0;

// objects
object objects[MAXOBJECTS];
std::string obj_patterns[MAXOBJECTS];
int objnum = 0;

xbeeAddr addresses[MAXMODIES];
unsigned char* instructions[MAXMODIES];
#ifdef DO_COMM
xbeecon *connections[MAXMODIES];

// xbee communication

struct xbee *xbee;
struct xbee_con *con;
struct xbee_conAddress address;
unsigned char txRet;
xbee_err ret;
unsigned char rotation[4] = {0x02,0x02,0x32,0x00};
pthread_t commthread;
#endif // DO_COMM
int cont;

// time marks
clock_t begin, end;  float time_spent;


static pthread_t tid;

// obtains the angle of a certain fiducial
double object_angle(double (*source)[4],double *pos)
{
    double rot_quat[4],dummy3[3];
    double inv[3][4];
    double sign;
    double angle;
    arUtilMat2QuatPos(source, rot_quat, pos);
    arUtilMatInv(source,inv);
    arUtilMat2QuatPos(inv, rot_quat, dummy3);
    sign = ((rot_quat[0]*rot_quat[1])>0)? 1 : -1;
    angle = sign*(rot_quat[1]*rot_quat[1])*180;
    return angle;
}

// set an arbitrary value in a certain position of the image
void setval(ARUint8 *dataPtr,int x, int y, unsigned char *value)
{
    if(x>0&&y>0&&x<image_width&&y<image_height)
    {
        dataPtr[image_width*3*y+x*3] = value[0];
        dataPtr[image_width*3*y+x*3+1] = value[1];
        dataPtr[image_width*3*y+x*3+2] = value[2];
    }
}
#ifdef DO_COMM
unsigned char ascii2hex(char ascii)
{
    switch(ascii)
    {
    case '0':
        return (unsigned char) 0;
    case '1':
        return (unsigned char) 1;
    case '2':
        return (unsigned char) 2;
    case '3':
        return (unsigned char) 3;
    case '4':
        return (unsigned char) 4;
    case '5':
        return (unsigned char) 5;
    case '6':
        return (unsigned char) 6;
    case '7':
        return (unsigned char) 7;
    case '8':
        return (unsigned char) 8;
    case '9':
        return (unsigned char) 9;
    case 'a':
    case 'A':
        return (unsigned char) 10;
    case 'b':
    case 'B':
        return (unsigned char) 11;
    case 'c':
    case 'C':
        return (unsigned char) 12;
    case 'd':
    case 'D':
        return (unsigned char) 13;
    case 'e':
    case 'E':
        return (unsigned char) 14;
    case 'f':
    case 'F':
        return (unsigned char) 15;
    default:
        return (unsigned char) 0;
    }
}


xbeeAddr parseAddr(char *src)
{
    int len = strlen(src);
    xbeeAddr outdir;
    memset(&outdir, 0, sizeof(outdir));
    outdir.addr64_enabled = 1;
    int addrc = 0;
    for(uint i = 0; i < (uint) len; i=i+2) outdir.addr64[addrc++] = ascii2hex(src[i])*16 + ascii2hex(src[i+1]);
    return outdir;
}

void setInstruction(unsigned char* array,uchar mode, uchar opt, uchar motorl, uchar motorr)
{
    array[0] = mode;
    array[1] = opt;
    array[2] = motorr;
    array[3] = motorl;
}
#endif // DO_COMM


int readModiConfig(char * filename)
{
    FILE *infile;
    infile = fopen(filename,"r");
    int head = 1;

    while(!feof(infile))
    {
        char strs[80];
        char strs2[80];
        if(head)
        {
            if(fscanf(infile,"%s",scenario)<0) return -1;
            if(fscanf(infile,"%s",modishape)<0) return -1;
            head = 0;
        }
        else
        {
            if(fscanf(infile,"%s %s",strs,strs2)<0)
            {
                if(modinum==0)  return -1;
                else return 0;
            }
            #ifdef DO_COMM
            addresses[modinum] = parseAddr(strs);
            #endif
            patterns[modinum] = std::string(strs2);
            modinum++;
        }
    }
    return 0;
}

int readObjConfig(char * filename)
{
    FILE *infile;
    infile = fopen(filename,"r");
    int head = 1;

    while(!feof(infile))
    {
        char strs[80];
        if(head)
        {
            if(fscanf(infile,"%s",objshape)<0) return -1;
            head = 0;
        }
        else
        {
            if(fscanf(infile,"%s",strs)<0)
            {
                if(objnum==0)  return -1;
                else return 0;
            }
            obj_patterns[objnum] = std::string(strs);
            objnum++;
        }
    }
    return 0;
}
#endif




