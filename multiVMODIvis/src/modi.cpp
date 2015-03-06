#include "modi.h"
#include "math.h"
#include <stdlib.h>

#define DEBUG 1
#define MINRANGE 35
#define DEFAULT_RANGE 100
#define DEFAULT_SPEED 60
// kalman filter strength
#define FQR 0.6

#define FRM_FLAG 1
#define COM_FLAG 2

unsigned char	red[] = {255,0,0};
unsigned char	green[] = {0,255,0};
unsigned char	blue[] = {0,0,255};

// constructors
modi::modi()
{
    #ifndef USE_SENSORS
    int id;
    data.id = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.pos.a = 0;
    data.flag = 0;
    #else
    data.flag = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.angl = 0;
    data.senc = DEFAULT_RANGE;
    data.senl = DEFAULT_RANGE;
    data.senr = DEFAULT_RANGE;
    data.speed = DEFAULT_SPEED;
    ofsx = 0;
    ofsy = 0;
    range = DEFAULT_RANGE;
    // virtual sensors
    scen = (pt*) malloc(sizeof(pt)*range);
    slef = (pt*) malloc(sizeof(pt)*range);
    srig = (pt*) malloc(sizeof(pt)*range);

    for(int i = 0; i < range; i++)
    {
        scen[i].x = ofsx + 0;
        scen[i].y = ofsy + i;
        srig[i].x = ofsx + -i;
        srig[i].y = ofsy + i;
        slef[i].x = ofsx + i;
        slef[i].y = ofsy + i;
    }
    #endif // USE_SENSORS
    patt_id = 0;
    k_index = 0;
    cf = 0;

    pfile = NULL;

    // kalman filter variables;
    th = 0;
    thp = 0;
    thp0 = 0;
    P = 320;
    Pp = 0;
    fQR = FQR;
    Q = fQR*100;
    R = 100;
    delta_thp = 0;
    thpp = 0;
    Kk = 0;

    scmd = (unsigned char*) malloc(sizeof(unsigned char)*4);
    memset(scmd,0x00,sizeof(unsigned char)*4);
}
modi::modi(int inoffx, int inoffy, int inrange)
{
    #ifndef USE_SENSORS
    int id;
    data.id = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.pos.a = 0;
    data.flag = 0;
    #else
    data.flag = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.angl = 0;
    data.senc = inrange;
    data.senl = inrange;
    data.senr = inrange;
    data.speed = DEFAULT_SPEED;

    ofsx = inoffx;
    ofsy = inoffy;
    range = inrange;

    scen = (pt*) malloc(sizeof(pt)*range);
    slef = (pt*) malloc(sizeof(pt)*range);
    srig = (pt*) malloc(sizeof(pt)*range);

    for(int i = 0; i < range; i++)
    {
        scen[i].x = ofsx + 0;
        scen[i].y = ofsy + i;
        srig[i].x = ofsx + -i;
        srig[i].y = ofsy + i;
        slef[i].x = ofsx + i;
        slef[i].y = ofsy + i;
    }
    #endif // USE_SENSORS

    patt_id = 0;
    k_index = 0;
    cf = 0;
    data.id = patt_id;

    pfile = NULL;

    // kalman filter variables;
    th = 0;
    thp = 0;
    thp0 = 0;
    P = 320;
    Pp = 0;
    fQR = FQR;
    Q = fQR*100;
    R = 100;
    delta_thp = 0;
    thpp = 0;
    Kk = 0;

    scmd = (unsigned char*) malloc(sizeof(unsigned char)*4);
    memset(scmd,0x00,sizeof(unsigned char)*4);
}
modi::modi(int inoffx, int inoffy, int inrange,int inpatt_id)
{
    #ifndef USE_SENSORS
    int id;
    data.id = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.pos.a = 0;
    data.flag = 0;
    #else
    data.flag = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.angl = 0;
    data.senc = inrange;
    data.senl = inrange;
    data.senr = inrange;
    data.speed = DEFAULT_SPEED;

    ofsx = inoffx;
    ofsy = inoffy;
    range = inrange;
    // virtual sensors
    scen = (pt*) malloc(sizeof(pt)*range);
    slef = (pt*) malloc(sizeof(pt)*range);
    srig = (pt*) malloc(sizeof(pt)*range);

    for(int i = 0; i < range; i++)
    {
        scen[i].x = ofsx + 0;
        scen[i].y = ofsy + i;
        srig[i].x = ofsx + -i;
        srig[i].y = ofsy + i;
        slef[i].x = ofsx + i;
        slef[i].y = ofsy + i;
    }
    #endif // USE_SENSORS
    patt_id = inpatt_id;
    data.id = patt_id;
    k_index = 0;
    cf = 0;

    if(DEBUG) printf("patt_id: %d\n",patt_id);

    pfile = NULL;

    // kalman filter variables;
    th = 0;
    thp = 0;
    thp0 = 0;
    P = 320;
    Pp = 0;
    fQR = FQR;
    Q = fQR*100;
    R = 100;
    delta_thp = 0;
    thpp = 0;
    Kk = 0;

    scmd = (unsigned char*) malloc(sizeof(unsigned char)*4);
    memset(scmd,0x00,sizeof(unsigned char)*4);
}
#ifdef USE_SENSORS
// updates the pixels which are inside the range of the sensors
// note that the angle between sensors is 45 degrees
void modi::updateSenPix()
{
    double ang = (2*M_PI/360)*(data.angl);
    double cosang = cos(ang);
    double sinang = sin(ang);
    for(int i = 0; i < range; i++)
    {

        scen[i].x = round(cosang*ofsx + sinang*(-i-ofsy)) + data.pos.x;
        scen[i].y = round(-sinang*ofsx + cosang*(-i-ofsy)) + data.pos.y;

        srig[i].x = round(cosang*(i+ofsx) + sinang*(-i-ofsy)) + data.pos.x;
        srig[i].y = round(-sinang*(i+ofsx) + cosang*(-i-ofsy)) + data.pos.y;

        slef[i].x = round(cosang*(-i+ofsx) + sinang*(-i-ofsy)) + data.pos.x;
        slef[i].y = round(-sinang*(-i+ofsx) + cosang*(-i-ofsy)) + data.pos.y;

    }
}
// updates the measured distance of every distance sensor of the robot
void modi::updateSDist(bmap bitmap)
{
    int lat_range = (int) round(range*0.71);
    for(int i = MINRANGE; i < range; i++)
    {
        if(bitmap.getWorking(scen[i].x,scen[i].y,0) ==255)
        {
            data.senc = i;
            break;
        }
        if(i == (range-1)) data.senc = range;
    }
    for(int i = MINRANGE; i < lat_range; i++)
    {
        if(bitmap.getWorking(slef[i].x,slef[i].y,0) ==255)
        {
            data.senl = (int) round(i*1.41);
            break;
        }
        if(i == (lat_range-1)) data.senl = range;
    }
    for(int i = MINRANGE; i < lat_range; i++)
    {
        if(bitmap.getWorking(srig[i].x,srig[i].y,0) ==255)
        {
            data.senr = (int) round(i*1.41);
            break;
        }
        if(i == (lat_range-1)) data.senr = range;
    }
}

// draws the sensors of this robot into the image with pointer dataPtr
void modi::drawSensorsInto(unsigned char *dataPtr,int width, int height)
{
    for(int i = 0; i < range; i++)
    {
        if(i< data.senc) setval(dataPtr, width, height, scen[i].x, scen[i].y,green);
        if(i< round(data.senl*0.71)) setval(dataPtr, width, height, slef[i].x, slef[i].y,red);
        if(i< round(data.senr*0.71)) setval(dataPtr, width, height, srig[i].x, srig[i].y,blue);
    }

}
#endif // USE_SENSORS
// initialize the logger for this modi WARNING: errors are not captured
void modi::initLogger(char *filename)
{
    pfile = fopen(filename,"w");
}
void modi::initTraceLogger(int idx)
{
    time_t rawtime;
    struct tm * timeinfo;
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    char strdate[80];
    sprintf(strdate,"%02d",idx);
    strftime(strdate+2,78,"_%Y-%m-%d_%I:%M:%S.txt",timeinfo);
    if(strdate!=NULL)
        tracefile = fopen(strdate,"w");
    else
        tracefile = NULL;
    printf("file %s ready to log.\n",strdate);
    fprintf(tracefile,"posx,posy,ang\n");
}
void modi::closeTraceLogger()
{
    fclose(tracefile);
}
void modi::updTraceLogger()
{
    if(tracefile!=NULL)
        fprintf(tracefile,"%.0f,%.0f,%.2f\n",data.pos.x,data.pos.y,data.pos.a);
}
// reports the current state of the robot
#ifdef USE_SENSORS
void modi::report(bool copy2file)
{
    printf("modi %02d: x:%d, y:%d, cf:%lf, kidx: %d, a:%f, sl:%d, sc: %d, sr: %d, speed:%d\n",patt_id,data.pos.x/(PPCM*100), data.pos.y/(PPCM*100), cf, k_index, data.angl, px2cm(data.senl), px2cm(data.senc), px2cm(data.senr),data.speed);
    if(copy2file)
        if(pfile!=NULL) fprintf(pfile, "%d,%d,%f,%d,%d,%d,%d\n",data.pos.x/(PPCM*100), data.pos.y/(PPCM*100), data.angl, data.senc, data.senc,data.senr,data.speed);
}
#else
void modi::report(bool copy2file)
{
    printf("modi %02d: x:%f, y:%f, cf:%lf, kidx: %d, a:%f\n",patt_id,data.pos.x/(PPCMX*100), data.pos.y/(PPCMY*100), cf, k_index, data.pos.a);
    if(copy2file)
        if(pfile!=NULL) fprintf(pfile, "%f,%f,%f\n",data.pos.x/(PPCMX*100), data.pos.y/(PPCMY*100), data.pos.a);
}
#endif
// set values into an image. value is an array of three uchar values
void modi::setval(unsigned char *dataPtr,int image_width, int image_height, int x, int y, unsigned char *value)
{
    if(x>0&&y>0&&x<image_width&&y<image_height)
    {
        dataPtr[image_width*3*y+x*3] = value[0];
        dataPtr[image_width*3*y+x*3+1] = value[1];
        dataPtr[image_width*3*y+x*3+2] = value[2];
    }
}
// checks if the robot was detected in the current frame (last frame captured)
bool modi::isVisible()
{
    return (k_index>-1);
}
// get the current pattern id for this robot
int modi::getPattid()
{
    return patt_id;
}
// getter and setter of the K index (fiducial indexes)
int modi::getKidx()
{
    return k_index;
}
void modi::setKidx(int value)
{
    k_index = value;
}
// getter and setter of the angle
double modi::getAngle()
{
    #ifndef USE_SENSORS
    return data.pos.a;
    #else // USE_SENSORS
    return data.angl;
    #endif // USE_SENSORS
};
void modi::setAngle(double angval)
{
    #ifndef USE_SENSORS
    data.pos.a = angval;
    #else // USE_SENSORS
    data.angl = angval;
    #endif
};
void modi::updateAngle(double angval)
{
    th = abs(angval);

    // time update
    delta_thp = thp-thp0;
    thp0 = thp;
    thpp = thp + delta_thp;
    Pp = P+Q;
    // measurement update
    Kk = (Pp)/(Pp+R);
    thp = thpp+Kk*(th-thpp);
    P = (1-Kk)*Pp;
    #ifndef USE_SENSORS
    data.pos.a = (angval>0) ? thp : -1*thp;
    #else
    data.angl = (angval>0) ? thp : -1*thp;
    #endif // USE_SENSORS

}
// getter and setter of the position of the robot
#ifndef USE_SENSORS
pta modi::getpos()
{
    return data.pos;
}
#else // USE_SENSORS
pt modi::getpos()
{
    return data.pos;
}
#endif // USE_SENSORS

void modi::drawTrace(bmap bitmap,int radius)
{
    int drawxc = (int)data.pos.x-bitmap.getOffx();
    int drawyc = (int)data.pos.y-bitmap.getOffy();

    for(int i=(drawxc-radius);i<(drawxc+radius);i++)
    {
        for(int j=(drawyc-radius);j<(drawyc+radius);j++)
        {
            if(i>=0&&j>=0&&i<=bitmap.getWidth()&&j<bitmap.getHeight()&&(abs((j-drawyc)*(j-drawyc))+abs((i-drawxc)*(i-drawxc)))<radius*radius)
            {
                setval(bitmap.getOriginalPtr(),bitmap.getWidth(),bitmap.getHeight(),i,j,red);
            }
        }
    }
}
void modi::setpos(int inx, int iny)
{
    data.pos.x = inx;
    data.pos.y = iny;
}
// set the current position of the robot plus the confidence
void modi::setpos(int inx, int iny,double incf)
{
    data.pos.x = inx;
    data.pos.y = iny;
    cf = incf;
}
// get the data of this robot
mDataf* modi::getmDatafref()
{
    return &data;
}

void modi::setshdata(int inkeydata,int inkeycmd)
{
    keydata = (key_t) inkeydata;
    if ((shmid = shmget(keydata, sizeof(mDataf), IPC_CREAT | 0666)) < 0)
    {
        perror("shmget");
        exit(1);
    }
    if ((sdata = (mDataf*) shmat(shmid, NULL, 0)) == (mDataf *) -1)
    {
        perror("shmat");
        exit(1);
    }
    keycmd = (key_t) inkeycmd;
    if ((shcmdid = shmget(keycmd, sizeof(unsigned char)*4, IPC_CREAT | 0666)) < 0)
    {
        perror("shmget");
        exit(1);
    }
    if ((scmd = (unsigned char*) shmat(shcmdid, NULL, 0)) == (unsigned char *) -1)
    {
        perror("shmat");
        exit(1);
    }
    scmd[0] = 0x02;
    scmd[1] = 0x01;
    scmd[2] = 0x00;
    scmd[3] = 0x00;

    printf("shared memories with keys %d and %d created.\n",keydata,keycmd);
}
void modi::updateshdata(bool comrdy)
{
    /*
    toggle the FRM_FLAG, to avoid that the controller send instructions when there's no
    new data available. For every new copied data, that bit is toggled, which means that
    there is new information available for the controller.
    */
    if(comrdy)
        data.flag = (data.flag&FRM_FLAG)^FRM_FLAG+COM_FLAG;
    else
        data.flag = (data.flag&FRM_FLAG)^FRM_FLAG;//+(data.flag&COM_FLAG)^;
    memcpy(sdata,&data,sizeof(mDataf));
}

void modi::setcmd(unsigned char *input)
{
    input[0] = scmd[0];
    input[1] = scmd[1];
    input[2] = scmd[2];
    input[3] = scmd[3];
}



void modi::update_act(float ml, float mr, float id)
{
    int pmotl = (int) (100.0*ml/(1.5*PI));
    int pmotr = (int) (100.0*mr/(1.5*PI));
    pmotl =     (pmotl>SPD_LIMIT)? SPD_LIMIT :
                (pmotl<-1*SPD_LIMIT)? -1*SPD_LIMIT : pmotl;
    pmotr =     (pmotr>SPD_LIMIT)? SPD_LIMIT :
                (pmotr<-1*SPD_LIMIT)? -1*SPD_LIMIT : pmotr;
    motl = (char) (pmotl>=0)? pmotl : -1*pmotl;
    motr = (char) (pmotr>=0)? pmotr : -1*pmotr;
    data.id = (int) id;

    scmd[0] = 0x02;
    scmd[1] =   (ml<0.0&&mr<0.0)?    0x00 :
                (ml<0.0&&mr>=0.0)?   0x01 :
                (ml>0.0&&mr<=0.0)?   0x02 :
                0x03;
    scmd[2] = motr;
    scmd[3] = motl;
    printf("id:%.0f, %d, %d,%f,%f\n",id,(int)motl,(int)motr,ml,mr);
}

void *modi_connection(void *arguments)
{
    modi* lmodi = (modi*) arguments;
   // flushing the console
    fflush(stdout);
    ///let the magic begin!
    int lsockid = lmodi->getSocket();
    float outdata[3];
    outdata[0] = lmodi->getpos().x/(PPCMX*100);
    outdata[1] = lmodi->getpos().y/(PPCMY*100);
    outdata[2] = lmodi->getpos().a;
    replyToReceivedData((char*)outdata,sizeof(float)*3, lmodi->getSocket());
    while(!shutdown_flag)
    {
        lmodi->swait();
        int receivedDataLength;
        char* receivedData=receiveData(receivedDataLength,lsockid);
        if (receivedData!=NULL)
        {
            // We received data. The server ALWAYS replies!
            // filling the allmodi_data array
            lmodi->update_act(((float*)receivedData)[0],-1*((float*)receivedData)[1],((float*)receivedData)[2]);
            // if(myid==0) simulation_time = ((float*)receivedData)[3];
            delete[] receivedData;
            outdata[0] = lmodi->getpos().x/(PPCMX*100)-1.18;
            outdata[1] = 0.68-lmodi->getpos().y/(PPCMY*100);
            outdata[2] = lmodi->getpos().a;
            //if(lmodi->getPattid()<=1) printf("modi %02d: x:%f, y:%f, a:%f\n",lmodi->getPattid(),outdata[0], outdata[1], outdata[2]);

            if (!replyToReceivedData((char*)outdata,sizeof(float)*3,lsockid))
            {
                printf("Failed to send reply.\n");
                break;
            }
        }
        else
        {
            printf("null data received. aborting...\n");
            break; // error
        }
    }
    printf("MODI %d disconnected.\n",lmodi->getPattid());
    close(lsockid);
    lmodi->sclose();
    return NULL;
}


void modi::setSocket(int i,modi *ref)
{
    printf("socket %d attached to modi %d.\n",i,patt_id);
    sockid = i;
    thrd = pthread_create(&tid,NULL,modi_connection,ref);
    pthread_detach(tid);
}

int modi::getSocket()
{
    return sockid;
}
