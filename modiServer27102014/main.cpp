#include "serverData.h"

int main(int argc, char** argv)
{
    char * env_filename = NULL;
    char * screen_size = NULL;
    char * frames_per_sec = NULL;
    static pthread_t tid;
    if(argc>1)
    {
        for(int i = 1; i< argc; i++)
        {
            if(strcmp(argv[i],"-env")==0)
                env_filename = argv[i+1];
            if(strcmp(argv[i],"-size")==0)
                screen_size = argv[i+1];
            if(strcmp(argv[i],"-fps")==0)
                frames_per_sec = argv[i+1];
            if(strcmp(argv[i],"-h")==0)
            {
                printf("usage: ./modiServer -env <environment filename> -size <window size> -fps <frames per second>\n");
                return 0;
            }
        }
    }
    if(env_filename==NULL)
        env_filename = (char *) "env_1.bmp";
    if(screen_size==NULL)
        screen_size = (char *) "600";
    if(frames_per_sec==NULL)
        frames_per_sec = (char *) "60";

    initServerData(env_filename,atoi(screen_size),atoi(frames_per_sec));

    int th1 = pthread_create(&tid,NULL,dataMonitor,NULL);
    int th2 = pthread_create(&tid,NULL,SDL_thread,NULL);
    int th3 = pthread_create(&tid,NULL,openModiThreads,NULL);
    int th4 = pthread_create(&tid,NULL,openObjsThreads,NULL);
    if (th1||th2||th3||th4)
    {
      printf("ERROR!"); exit(-1);
    }
    pthread_join(tid,NULL);
    return 0;
}
