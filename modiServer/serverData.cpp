#include "serverData.h"

/* simulation related variables */

int     shutdown_flag;
float   simulation_time;
float   simulation_time_t;

/* modies and objects storage */

smodi   all_modies[MAX_MODIES];
int     modiCounter;
mData   allobjects_dataFROMvrep[MAX_OBJECTS];
int     objCounter;

/* semaphores variables */

sem_t   sem_writer;
sem_t   sem_reader;
sem_t   sem_display;
sem_t   mutex;
int     callCounter;

/* SDL-related variables */

SDL_Event       event;
SDL_Window      *mwindow;
SDL_Surface     *window_surface;
surf_wrapper    back_surf;
surf_wrapper    modi_surf;
surf_wrapper    objt_surf;
pta             ScaleBg;
pta             ScaleFactors;
SDL_Rect        displayrect;
SDL_Rect        bgrect;

char *          env_name;
int             c_fps;
int             c_winsize;


/* all the functions below */

void wait_for_sync()
{
    sem_wait(&sem_reader);
    if(SEM_DEBUG)
    {
        printf("R");
        fflush(stdin);
    }
}
void reset_sync()
{
    sem_wait(&mutex);
    int max = callCounter;
    for(int i = 0; i<max; i++)
        sem_post(&sem_writer);
    callCounter = 0;
    if(SEM_DEBUG)
    {
        printf("D\n");
        fflush(stdin);
    }
    sem_post(&mutex);
}
void wait_for_usage()
{
    sem_wait(&mutex);
    callCounter++;
    if(callCounter>=(modiCounter+objCounter))
    {
        if(SEM_DEBUG)
        {
            printf("!");
            fflush(stdin);
        }
        sem_post(&sem_reader);
    }
    else if(SEM_DEBUG)
    {
    }
    sem_post(&mutex);
    sem_wait(&sem_writer);
}
void * modi_connection(void *arguments)
{
    // getting my own ID
    int myid = modiCounter-1;
    all_modies[myid].init(myid,ScaleBg,bgrect);
    // flushing the console
    fflush(stdout);
    // accepted returned connection
    int incoming_connection = *((int *) arguments);
    printf("modi connected with the ID %d.",myid);
    float outdata[3];
    char semname[6];
    sprintf(semname,"sem%02d",myid);
    sem_t *exesem = sem_open(semname,O_CREAT,0644,1);
    printf(" semaphore %s was created.\n",semname);

    ///let the magic begin!
    while(!shutdown_flag)
    {
        // This is the server loop (the robot's control loop):
        //sem_wait(&sem_writer);
        int receivedDataLength;
        char* receivedData=receiveData(receivedDataLength,incoming_connection);
        if (receivedData!=NULL)
        {

            // We received data. The server ALWAYS replies!
            // filling the allmodi_data array
            all_modies[myid].upd_data_from_vrep(((float*)receivedData)[0],((float*)receivedData)[1],((float*)receivedData)[2]);

            if(myid==0) simulation_time = ((float*)receivedData)[3];
            delete[] receivedData;

            outdata[0] = all_modies[myid].get_data_to_vrep().left;
            outdata[1] = all_modies[myid].get_data_to_vrep().right;

            outdata[2] = myid;
            if (!replyToReceivedData((char*)outdata,sizeof(float)*3,incoming_connection))
            {
                printf("Failed to send reply.\n");
                break;
            }
          //  sem_post(&sem_reader);
            sem_post(exesem);
            wait_for_usage();

        }
        else
            break; // error
    }
    printf("client ID %d disconnected.\n",myid);
    close(incoming_connection);
    modiCounter = 0;
    simulation_time_t = 0;
    all_modies[myid].make_available();
    shutdown_flag = 1;
    objCounter = 0;
    allobjects_dataFROMvrep[myid].id = -1;
    sem_close(exesem);
    return NULL;
}
void * obj_connection(void *arguments)
{
    // getting my own ID
    int myid = objCounter-1;
    allobjects_dataFROMvrep[myid].id = myid;
    // flushing the console
    fflush(stdout);
    // accepted returned connection
    int incoming_connection = *((int *) arguments);
    printf("object detected with the ID %d\n",myid);
    char outdata;

    ///let the magic begin!
    while(!shutdown_flag)
    {
        // This is the server loop (the robot's control loop):
        //sem_wait(&sem_writer);
        int receivedDataLength;
        char* receivedData=receiveData(receivedDataLength,incoming_connection);
        if (receivedData!=NULL)
        {

            // We received data. The server ALWAYS replies!
            // filling the allmodi_data array
            allobjects_dataFROMvrep[myid].pos.x = ((float*)receivedData)[0];
            allobjects_dataFROMvrep[myid].pos.y = ((float*)receivedData)[1];
            allobjects_dataFROMvrep[myid].pos.a = ((float*)receivedData)[2];
            delete[] receivedData;
            // filling the outdata array
            outdata = (char) myid;
            if (!replyToReceivedData(&outdata,sizeof(char),incoming_connection))
            {
                printf("Failed to send reply.\n");
                break;
            }
            //sem_post(&sem_reader);
            wait_for_usage();
        }
        else
            break; // error
    }
    printf("object ID %d disconnected.\n",myid);
    close(incoming_connection);
    modiCounter = 0;
    simulation_time_t = 0;
    all_modies[myid].make_available();
    shutdown_flag = 1;
    objCounter = 0;
    allobjects_dataFROMvrep[myid].id = -1;
    return NULL;
}

void initServerData(char *env, int winsize, int fps)
{
    env_name = env;
    c_winsize = winsize;
    c_fps = fps;

    shutdown_flag = 0;
    modiCounter = 0;
    objCounter = 0;

    for(int i = 0; i < MAX_MODIES; i++) all_modies[i] = smodi();
    for(int i = 0; i< MAX_OBJECTS; i++)
    {
        allobjects_dataFROMvrep[i].id = -1;
        allobjects_dataFROMvrep[i].pos.x = 0;
        allobjects_dataFROMvrep[i].pos.y = 0;
        allobjects_dataFROMvrep[i].pos.a = 0;
    }
    if(sem_init(&sem_reader, 0, 0)<0)  printAndExit((char *)"error! sem_reader.");
    if(sem_init(&sem_writer, 0, MAXCONNECTIONS)<0) printAndExit((char *)"error! sem_writer.");
    if(sem_init(&sem_display, 0, 0)<0) printAndExit((char *)"error! sem_display.");
    if(sem_init(&mutex, 0, 1)<0)       printAndExit((char *)"error! mutex.");
}
void printData(int what)
{
    for(int i = 0; i < MAX_MODIES; i++)
    {
        if(!all_modies[i].isEmpty())
            all_modies[i].printUtil(what);
    }
    printf("\n");
}
void *openModiThreads(void *arg)
{
    pthread_t tid;
    int s = simpleReuseSocket(PORTNUMBER);
    while(modiCounter < (MAXCONNECTIONS-1))
    {
        int	incoming_connection = simpleAcceptConn(s);
        if(modiCounter==0) shutdown_flag = 0;
        modiCounter++;
        int rc = pthread_create(&tid,NULL, modi_connection, (void *) &incoming_connection);
        if (rc)
        {
            printf("ERROR; return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
        pthread_detach(tid);
    }
    return NULL;
}
void *openObjsThreads(void *arg)
{
    pthread_t tid;
    int o = simpleReuseSocket(PORTNUMBER+1);
    while(objCounter < (MAXCONNECTIONS-1))
    {
        int	incoming_connection = simpleAcceptConn(o);
        objCounter++;
        int rc = pthread_create(&tid,NULL, obj_connection, (void *) &incoming_connection);
        if (rc)
        {
            printf("ERROR; return code from pthread_create() is %d\n", rc);
            exit(-1);
        }
        pthread_detach(tid);
    }
    return NULL;
}
void easyTest()
{
    float spd_factor = 2;
    int startModi = 1;
    if (simulation_time-simulation_time_t<4.0f)
    {
        // driving backwards while slightly turning:
        if(simulation_time-simulation_time_t<1.5f)
        {

            for(int i=startModi; i<MAX_MODIES; i++)
            {
                if(!all_modies[i].isEmpty())
                    all_modies[i].setMotors(-PI*0.5f,-PI*0.25f);
            }
        }
        else
        {
            for(int i=startModi; i<MAX_MODIES; i++)
            {
                if(!all_modies[i].isEmpty())
                    all_modies[i].setMotors(PI*spd_factor,-PI*spd_factor);
            }
        }
    }
    else
    {
        for(int i=startModi; i<MAX_MODIES; i++)
        {
                if(!all_modies[i].isEmpty())
                    all_modies[i].setMotors(-PI*0.5f,-PI*0.25f);
        }
         simulation_time_t=simulation_time;
    }


}
void quit(int rc)
{
    modi_surf.~surf_wrapper();
    back_surf.~surf_wrapper();
    SDL_DestroyWindow(mwindow);
    mwindow = NULL;
    SDL_Quit();
    exit(rc);
}
void checkEvent(int *done)
{
    while (SDL_PollEvent(&event))
    {
        switch (event.type)
        {
        case SDL_WINDOWEVENT:
            if (event.window.event == SDL_WINDOWEVENT_RESIZED)
            {
                window_surface = SDL_GetWindowSurface(mwindow);
                displayrect.w = event.window.data1;
                displayrect.h = event.window.data2;
            }
            break;
        case SDL_KEYDOWN:
            if (event.key.keysym.sym != SDLK_ESCAPE) break;
        case SDL_QUIT:
            (*done) = SDL_TRUE;
            break;
        }
    }
}
void drawModi(smodi modi)
{
    SDL_Rect temp;
    temp = modi.getImgPos();
    modi_surf.rot(modi.get_pos().a*PI/180);
    if(!SDL_BlitScaled(modi_surf.get(WORKING), NULL, back_surf.get(WORKING), &temp))
    {
        SDL_GetError();
    }
}
void *dataMonitor(void *arguments)
{
    printf("\twaiting for users...\n");
    fflush(stdout);
    while(1)
    {
        wait_for_sync();
        //sem_wait(&sem_reader);
        sem_wait(&sem_display);
        // do something
        for(int i=0; i<MAX_MODIES; i++)
        {
            if(!all_modies[i].isEmpty())
            {
                all_modies[i].upd_motor_from_shm();
                all_modies[i].upd_sensors_shm();
            }
        }
        //easyTest();
        //printData(OBJ_DISTANCES);
        //sem_post(&sem_writer);
        reset_sync();
    }
    return NULL;
}
void printAndExit(char *what)
{
    printf( "%s SDL Error: %s\n", what, SDL_GetError() );
    quit(4);
}
void drawObjects()
{
    for(int i=0; i<MAX_OBJECTS; i++)
    if(allobjects_dataFROMvrep[i].id>=0)
    {

        SDL_Rect tmp;
        tmp.w = OBJ_WIDTH/ScaleBg.x;
        tmp.h = OBJ_HEIGHT/ScaleBg.y;
        tmp.x = (int) (bgrect.w/2 + allobjects_dataFROMvrep[i].pos.x/ScaleBg.x-0.5*OBJ_WIDTH/ScaleBg.x);
        tmp.y = (int) (bgrect.h/2 - allobjects_dataFROMvrep[i].pos.y/ScaleBg.y-0.5*OBJ_WIDTH/ScaleBg.y);
        if(!SDL_BlitScaled(objt_surf.get(WORKING), NULL, back_surf.get(WORKING), &tmp))
    {
        SDL_GetError();
    }
    }
}
void* SDL_thread(void * args)
{
    displayrect.w = c_winsize;
    displayrect.h = c_winsize;
    //Initialize SDL and windows
    if(SDL_Init(SDL_INIT_VIDEO)<0) printAndExit((char*) "SDL could not be init!");
    mwindow = SDL_CreateWindow((char*) "MODI VIEWER", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, c_winsize, c_winsize,SDL_WINDOW_RESIZABLE+ SDL_WINDOW_SHOWN );
    if(mwindow==NULL) printAndExit((char*) "Window could not be created!");
    window_surface = SDL_GetWindowSurface(mwindow);

    modi_surf.init((char*) "icon.bmp",window_surface,true);
    back_surf.init(env_name,window_surface);
    objt_surf.init((char*) "circle.bmp",window_surface,true);
    bgrect = back_surf.get_size();

    ScaleBg.x = SCENARIO_SIZE/bgrect.w;
    ScaleBg.y = SCENARIO_SIZE/bgrect.h;

    printf("virtual scenario size: (%d,%d), scale: %.3f [m/pix]\n",bgrect.w,bgrect.h,ScaleBg.x);

    int done = 0;
    while (!done)
    {
        /* pre-updating operations */
        checkEvent(&done);
        back_surf.reset();
        /* first iteration: draw modies and objects */
        drawObjects();
        for(int i=0; i<MAX_MODIES; i++)
            if(!all_modies[i].isEmpty())
            {
                drawModi(all_modies[i]);
            }
        /* second iteration: update all the sensors */
        for(int i=0; i<MAX_MODIES; i++)
            if(!all_modies[i].isEmpty())
            {
                all_modies[i].upd_distances_to_modies(all_modies,&back_surf);
                all_modies[i].upd_distances_to_objects(allobjects_dataFROMvrep,&back_surf);
                all_modies[i].upd_prox_sen(&back_surf);
                all_modies[i].upd_col_sen(&back_surf);
                all_modies[i].upd_mo_array(all_modies);
            }
        /* third iteration: update all the remaining draws */
        for(int i=0; i<MAX_MODIES; i++)
            if(!all_modies[i].isEmpty())
            {
                all_modies[i].draw_prox_sen(&back_surf);
                all_modies[i].draw_col_sen(&back_surf);
                /* wanna draw also the rays between objects/modies ? */
                //all_modies[i].draw_rays(&back_surf);
            }
        /* post_updating operations */
        sem_post(&sem_display);
        SDL_Delay(1000/c_fps);
        SDL_BlitScaled(back_surf.get(WORKING), NULL, window_surface, NULL);
        SDL_UpdateWindowSurface( mwindow );
    }
    quit(0);
    return NULL;
}
