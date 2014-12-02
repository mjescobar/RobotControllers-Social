#include "vmodi.h"
#include "socket_functions.h"
#include <time.h>
#include <vector>

bmap b1;
bmap shape;

int fcounter;

//debug
int inscounter = 0;
bool insflag = true;

#ifdef DO_COMM
unsigned char nomov[4] = {0x02,0x01,0x00,0x00};

// show the received data in the xbee module
void myCB(struct xbee *xbee, struct xbee_con *con, struct xbee_pkt **pkt, void **data)
{
    if ((*pkt)->dataLen == 0)
    {
        printf("too short...\n");
        return;
    }
    printf("rx: [%s]\n", ((*pkt)->data));
}
// to be used within a process
void *sendcommand(void *arg)
{
    for(int i = 0; i < (int) modinum; i++)
        ret = xbee_connTx(connections[i], NULL, instructions[i],4);
    return NULL;
}
#endif // DO_COMMqq

int main(int argc, char **argv)
{
    printf("starting!\n");
    int dummy = 1;
    glutInit(&dummy,NULL);
    init((char *) "Data/config.txt",NULL);

    fcounter = -1;

    arVideoCapStart();
    argMainLoop( NULL, keyEvent, mainLoop );

    #ifdef DO_COMM
    if ((ret = xbee_conEnd(con)) != XBEE_ENONE) xbee_errorToStr(ret);
    xbee_shutdown(xbee);
    #endif

    return (0);
}

static void keyEvent( unsigned char key, int x, int y)
{
    if( key == 0x1b )
    {
        #ifdef DO_COMM
        for(int i = 0; i < (int) modinum; i++)
            ret = xbee_connTx(connections[i], NULL, nomov,4);
        #endif // DO_COMM
        printf("*** %f (frame/sec)\n", (double)count/arUtilTimer());
        stop_server();
        cleanup();
        exit(0);
    }
    if( key == 's' )
    {
        toggle_reg  = !toggle_reg;
        if(toggle_reg)
        {
            for(int i = 0; i < (int) modinum; i++)
                modies[i].initTraceLogger(i);

            send_msg((char*)"start");
        }
        else
        {
            for(int i = 0; i < (int) modinum; i++)
                modies[i].closeTraceLogger();
        }
        b1 = bmap(scenario,10,40,image_width,image_height);
        b1.clearWorking();


    }
}
/* main loop */
static void mainLoop(void)
{
    ARUint8         *dataPtr;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    int             j;
    double 	    patt_trans1[3][4];


    /* grab a video frame */
    if( (dataPtr = (ARUint8 *)arVideoGetImage()) == NULL )
    {
        arUtilSleep(2);
        return;
    }
    if( count == 0 ) arUtilTimerReset();
    count++;

    /* detect the markers in the video frame */
    if( arDetectMarker(dataPtr, thresh, &marker_info, &marker_num) < 0 )
    {
        cleanup();
        exit(0);
    }
    // a copy of the frame is used to avoid flickering of the overlayed image
    memcpy(pframe, dataPtr,b1.getFramesize());
    // clear the current working copy of the background, i.e, restore the original image.
    b1.clearWorking();
    int q;


        // draws position of every robot
    for(q= 0; q < modinum; q++)
    {
        if(modies[q].isVisible())
        {
            shape.overlaySqFloating(b1,(int)modies[q].getpos().x, (int) modies[q].getpos().y);
        }
    }


    // virtual sensors section: updating and drawing
    #ifdef USE_SENSORS
    for(q = 0; q < modinum; q++)
    {
        modies[q].updateSenPix();
        modies[q].updateSDist(b1);
    }
    for(q = 0; q < modinum; q++)
    {
        if(modies[q].isVisible())
        {
            modies[q].drawSensorsInto(pframe,image_width,image_height);
        }
    }
    #endif
    if(toggle_reg)
    {
        for(q = 0; q < modinum; q++)
        {
            if(modies[q].isVisible())
            {
                modies[q].drawTrace(b1,3);
                modies[q].updTraceLogger();
            }
        }
    }
    b1.overlayInto(pframe,image_width,true);


    argDrawMode2D();
    argDispImage( pframe, 0,0 );
    arVideoCapNext();

    /* check for object visibility */
    for(int i = 0; i < (int) modinum; i++)
    {
        modies[i].setKidx(-1);
        for( j = 0; j < marker_num; j++ )
        {
            if( modies[i].getPattid() == marker_info[j].id )	if( modies[i].getKidx() == -1 ) modies[i].setKidx(j);
        }
        /* get the transformation between the marker and the real camera */
        if(modies[i].isVisible())
        {
            int cPattIndex = modies[i].getKidx();
            arGetTransMat(&marker_info[cPattIndex], patt_center, patt_width, patt_trans1);
            /* updating position and angle */
            int xpos = (int) round(marker_info[cPattIndex].pos[0]);
            int ypos = (int) round(marker_info[cPattIndex].pos[1]);
            modies[i].setpos(xpos,ypos,marker_info[cPattIndex].cf);
            /* this code adds a running average in the lecture of the sensors */
            /*
            float fading = 0.3;
            modies[i].setAngle(fading*modies[i].getAngle() + (1-fading)*object_angle(patt_trans1,marker_info[cPattIndex].pos));
            */
            modies[i].updateAngle(object_angle(patt_trans1,marker_info[cPattIndex].pos));
            // this ensures the correct reception of the command ((FRAMES_PER_COM-1) frames in advanced)
            if(cont==FRAMES_PER_COM-2) modies[i].updateshdata(true);
            else modies[i].updateshdata(false);
        }
    }

    /* every FRAMES_PER_COM frames, a control signal is sent to the robot. or instead, the value of the virtual sensors */
    if(cont==FRAMES_PER_COM)
    {
        #ifdef DO_COMM
        for(int i = 0; i < (int) modinum; i++)
        {
            modies[i].setcmd(instructions[i]);
        }
        if(pthread_create(&commthread, NULL,sendcommand,NULL)) fprintf(stderr, "Error creating thread\n");
        #endif // DO_COMM
        #ifdef DO_REPORT
        for(int i = 0; i < (int) modinum; i++)  modies[i].report(true);
        printf("\n");
        #endif // DO_REPORT
    }
    cont++;
    for(int i = 0; i < (int) modinum; i++)  modies[i].report(false);
    cont = 0;
    argSwapBuffers();
    if(fcounter != -1) printf(" fc%4d\n",fcounter);
    if(fcounter>0) fcounter--;
    if(fcounter==0)
    {
        cleanup();
        exit(0);
    }
}

static void init( char * infilename, char *outfilename )
{
    time_spent = 0;
    /* setup of ARToolkit */
    ARParam  wparam;
    if( arVideoOpen( vconf ) < 0 ) exit(0);
    if( arVideoInqSize(&xsize, &ysize) < 0 ) exit(0);
    printf("Image size (x,y) = (%d,%d)\n", xsize, ysize);
    if( arParamLoad(cparam_name, 1, &wparam) < 0 )
    {
        printf("Camera parameter load error !!\n");
        exit(0);
    }
    arParamChangeSize( &wparam, xsize, ysize, &cparam );
    arInitCparam( &cparam );
    printf("*** Camera Parameter ***\n");
    arParamDisp( &cparam );

    if(readModiConfig(infilename)<0)
    {
        printf("modinum: %d error! exiting...\n",modinum);
        exit(0);
    }
    // setup of Modi and virtual representation
    for(int i = 0; i < modinum; i++)
    {
        modies[i] = modi(0,cm2px(2),cm2px(50),arLoadPatt(patterns[i].c_str())); // 50 cm sensor range
        modies[i].setshdata(32000+i,32000+i+MAXMODIES);
    }
    for(int i = 0; i < modinum; i++)
    {
        if(modies[i].getPattid() <0)
        {
            printf("error loading pattern for modi n.%d !!\nexiting...\n",i);
            exit(0);
        }
        char logfilename[100];
        if(outfilename!=NULL) sprintf(logfilename,"%s_modi%02d.txt",outfilename,i);
        else sprintf(logfilename,"logmove_modi%02d.txt",i);
        modies[i].initLogger(logfilename);
        instructions[i] = (unsigned char*) malloc(sizeof(unsigned char)*4);
    }
    printf("%d MODIes were created.\n",(int)modinum);

    // change here if you want to change the position of the scenario in the screen
    b1 = bmap(scenario,10,40,image_width,image_height);
    b1.clearWorking();
    shape = bmap(modishape,image_width,image_height,image_width,image_height);

    pframe = (unsigned char *) malloc(sizeof(unsigned char)*b1.getFramesize());

    /* open the graphics window */

    printf("starting artoolkit window... ");
    argInit( &cparam, 1.0, 0, 0, 0, 0 );
    printf("done.\n");
    cont = 0;

#ifdef DO_COMM
    printf("reach state");
    fflush(stdin);
    if ((ret = xbee_setup(&xbee, "xbeeZB", "/dev/ttyUSB0", 57600)) != XBEE_ENONE) xbee_errorToStr(ret);
    for(int i = 0; i < modinum; i++)
    {
        if ((ret = xbee_conNew(xbee, &connections[i], "Data",&addresses[i])) != XBEE_ENONE) xbee_errorToStr(ret);
        if ((ret = xbee_conCallbackSet(connections[i], myCB, NULL)) != XBEE_ENONE) xbee_errorToStr(ret);
        ret = xbee_connTx(connections[i], NULL, nomov,4);
    }
#endif
    printf("starting server... ");
    fflush(stdin);
    start_server();
    printf("done\n");
}
/* cleanup function called when program exits */
static void cleanup(void)
{
    arVideoCapStop();
    arVideoClose();
    argCleanup();
}
