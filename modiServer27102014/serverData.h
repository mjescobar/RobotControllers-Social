#ifndef SERVERDATA_H_INCLUDED
#define SERVERDATA_H_INCLUDED

#include "smodi.h"
#include "comm_utils.h"

/* simulation definitions */

#define SEM_DEBUG 0         /* wanna debug the semaphores? */
#define SCENARIO_SIZE 5.0   /* scenario size (in meters */

/* objects definitions */

#define OBJ_WIDTH 0.1       /* objects width */
#define OBJ_HEIGHT 0.1      /* objects height */

void wait_for_sync();
void reset_sync();
void wait_for_usage();

void * modi_connection(void *arguments);
void * obj_connection(void *arguments);

void initServerData(char *env, int winsize, int fps);
void printData(int what);
void *openModiThreads(void *arg);
void *openObjsThreads(void *arg);
void easyTest();
void quit(int rc);
void checkEvent(int *done);
void drawModi(smodi modi);
void *dataMonitor(void *arguments);
void printAndExit(char *what);
void drawObjects();
void* SDL_thread(void * args);

#endif // SERVERDATA_H_INCLUDED
