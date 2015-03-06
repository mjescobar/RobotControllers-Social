#include <iostream>
#include "include/perception.h"
#include "include/motiv.h"
#include "include/behaviorsh.h"
#include "include/behexec.h"


/* semaphore-related includes */
#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

using namespace std;

ModiSen r1;
perception perc;
motiv mobl;
behaviorsh besh;
behexec executor;


float absv(float x)
{
    float o = (x>=0)? x: -1*x;
    return o;
}

void prt_evt(int *evt)
{
    string str_evt[] =
    {
        "NULL_EVENT",
        "COL_FRONT",
        "COL_BACK",
        "COL_LEFT",
        "COL_RIGHT",
        "NEAR_LEFT",
        "NEAR_CENTER",
        "NEAR_RIGHT",
        "CORNER_LEFT",
        "CORNER_RIGHT",
        "OBJ_DETECT",
        "MOD_DETECT",
        "OBJ_TOUCH",
        "MOD_TOUCH"
    };
    cout<< "== events: =="<<endl;
    for(int i=0; i < MAX_EVENTS; i++)
    {
        if(evt[i]) cout<<str_evt[i]<<endl;
    }
    cout<< "============="<<endl;
}

void prt_beh(int idx)
{
    string str_beh[] =
    {
        "FULL_STOP",
        "WALL_FOLLOW",
        "REACH_OBJ",
        "EVADE_OBJ",
        "FREE_EXP",
        "MARK_CORNERS",
        "TURN_180",
        "MODI_FOLLOW",
        "MORE_SPD",
        "LESS_SPD",
        "EVADE_OBST"
    };
    //cout << "== behaviors: =="<<endl;
    if(idx==-1) cout << "NO_BEHAVIOR"<<endl;
    else cout << str_beh[idx]<<endl;
    //cout << "================"<<endl;
}

void prt_ModiSen(ModiSen r)
{
    printf("modi id %d, mot.orientation: %.2f\n\t(xpos,ypos,ang):\n\t\t(%.3f,%.3f,%.3f)\n",r.id, r.mo,r.pos.x, r.pos.y, r.pos.a);
    printf("\tprox.sen:\n\t\t(%.3f,%.3f,%.3f)\n",r.senl, r.senc, r.senr);
    printf("\tcol.sen:\n\t\t");
    for(int i=0; i<NCSENS; i++) printf("%d",r.colsen[i]);
    printf("\n\tvisible modies (id,dist,ang):\n\t\t");
    for(int i=0; i<MAX_MODIES; i++)
        if(r.m_pos[i].m>0) printf("(%d,%.3f,%.1f) ",i,r.m_pos[i].m,r.m_pos[i].a);
    printf("\n\tvisible objects (id,dist,ang):\n\t\t");
    for(int i=0; i<MAX_OBJECTS; i++)
        if(r.o_pos[i].m>0) printf("(%d,%.3f,%.1f) ",i,r.o_pos[i].m,r.o_pos[i].a);
    printf("\n\tmotivational orientations (id,value):\n\t\t");
    for(int i=0; i<MAX_MODIES; i++)
        if(r.m_pos[i].m>0) printf("(%d,%.2f) ",i,r.mo_sur[i]);
    printf("\n\n");
}

void prt_header(FILE *outf)
{
    fprintf(outf,"mo,posx,posy,angle,psenl,psenc,psenr,nmodies,nobjects,");
    for(int i=0; i<NCSENS; i++)
        fprintf(outf,"colsen%02d,",i);
    fprintf(outf,"EO,currbeh,evt00");
    for(int i=1; i < MAX_EVENTS; i++)
        fprintf(outf,",evt%02d",i);
    fprintf(outf,"\n");
}

void prt_all(FILE *outf, ModiSen r, float EO, int *evt, int beh)
{
    int sum1 = 0;
    int sum2 = 0;
    for(int i=0; i<MAX_MODIES; i++)
        if(r.m_pos[i].m>0) sum1++;
    for(int i=0; i<MAX_OBJECTS; i++)
        if(r.o_pos[i].m>0) sum2++;
    fprintf(outf,"%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,",r.mo,r.pos.x,r.pos.y,r.pos.a,r.senl,r.senc,r.senr,sum1,sum2);
    for(int i=0; i<NCSENS; i++)
        fprintf(outf,"%d,",r.colsen[i]);
    fprintf(outf,"%.2f,%d",EO,beh);
    for(int i=0; i < MAX_EVENTS; i++)
        fprintf(outf,",%d",evt[i]);
    fprintf(outf,"\n");
}

void init(int idx,float inMO, float *inweights)
{
    r1.id = idx;
    r1.senl = DEFAULT_RANGE;
    r1.senc = DEFAULT_RANGE;
    r1.senr = DEFAULT_RANGE;
    for(int i=0; i<NCSENS; i++) r1.colsen[i] = 0;
    for(int i=0; i<MAX_MODIES; i++) r1.m_pos[i].m = -1;
    for(int i=0; i<MAX_MODIES; i++) r1.mo_sur[i] = 0;
    for(int i=0; i<MAX_OBJECTS; i++) r1.o_pos[i].m = -1;
    r1.mo = inMO;
    perc.setSens(&r1);
    perc.initShm(r1.id);
    mobl.reset(inMO,inweights);
    perc.reset(0,MAX_EVENTS);
    executor.setSens(&r1);
    executor.initShm(r1.id);
}

int main(int argc, char** argv)
{
    char* c_idx = NULL;
    char* c_inMO = NULL;
    char* c_delay = NULL;
    char* c_beh = NULL;
    char* c_modi2follow = NULL;
    char* c_obj2follow = NULL;
    float* inweights = NULL;
    int csv_make = 0;
    long int limit = 0;
    long int counter = 1;
    FILE *csv_file = NULL;
    // initialization
    if(argc>1)
    {
        for(int i=1; i<argc; i++)
        {
            if(strcmp(argv[i],"-idx")==0)
                c_idx=argv[i+1];
            if(strcmp(argv[i],"-mo")==0)
                c_inMO=argv[i+1];
            if(strcmp(argv[i],"-delay")==0)
                c_delay=argv[i+1];
            if(strcmp(argv[i],"-beh")==0)
                c_beh=argv[i+1];
            if(strcmp(argv[i],"-mfol")==0)
                c_modi2follow=argv[i+1];
            if(strcmp(argv[i],"-ofol")==0)
                c_obj2follow=argv[i+1];
            if(strcmp(argv[i],"-limit")==0)
                limit=atoi(argv[i+1]);
            if(strcmp(argv[i],"-csv")==0)
            {
                csv_make = 1;
                csv_file = fopen(argv[i+1],"w");
            }
            if(strcmp(argv[i],"-mow")==0)
            {
                inweights = (float*) malloc(sizeof(float)*3);
                inweights[0]=atof(argv[i+1]);
                inweights[1]=atof(argv[i+2]);
                inweights[2]=atof(argv[i+3]);
            }

            if(strcmp(argv[i],"-h")==0)
            {
                printf("usage: ./proto_arch -idx <modi id> -mo <initial MO> -delay <time before start> -beh <selected behavior> -mfol <modi to follow> -ofol <object to follow> -mow <w_mo> <w_bf> <w_eo>\n");
                return 0;
            }
        }
    }
    if(c_idx==NULL)
        c_idx = (char *)"0";
    if(c_inMO==NULL)
        c_inMO = (char *)"0.0";
    if(c_delay==NULL)
        c_delay =(char *)"5";
    if(c_modi2follow==NULL)
        c_modi2follow =(char *)"-2";
    if(c_obj2follow==NULL)
        c_obj2follow =(char *)"-2";
    if(inweights==NULL)
    {
        inweights = (float*) malloc(sizeof(float)*3);
        inweights[0] = 0.992; /// M_MO
        inweights[1] = 0.3 ; /// M_BF
        inweights[2] = 0.7; /// M_EO
    }
    printf("init:\n\tid:%d\n\tmo:%.2f\n\tweights: w_mo %.4f, w_bf %.2f, w_eo %.2f\nwaiting for the start!\n",atoi(c_idx),atof(c_inMO),inweights[0], inweights[1], inweights[2]);

    init(atoi(c_idx),atof(c_inMO),inweights);
    int modi2follow = atoi(c_modi2follow);
    int obj2follow = atoi(c_obj2follow);

    executor.reset(modi2follow,obj2follow);
    executor.update(FULL_STOP);
    executor.upd_motor_and_mo_to_shm(mobl.GetMO());
    for(int q = atoi(c_delay); q >=0; q--)
    {
        usleep(1000000);
    }

    printf("\nsimulation started! - ver 16122014\n");

    /*  MAIN LOOP */

    if(csv_make)
    {
        time_t rawtime;
        struct tm * timeinfo;
        char buffer[80];
        time (&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
        std::string str(buffer);

        fprintf(csv_file,"%s\ninit:\n\tid:%d\n\tmo:%.2f\n\tweights: w_mo %.4f, w_bf %.2f, w_eo %.2f\n",str.c_str(),atoi(c_idx),atof(c_inMO),inweights[0], inweights[1], inweights[2]);
        prt_header(csv_file);
    }
    int div = limit/100;

    char semname[6];
    printf("opening sem...\n");
    sprintf(semname,"sem%02d",atoi(c_idx));
    sem_t *exesem = sem_open(semname,0);
    if(exesem==SEM_FAILED||exesem==NULL)
        printf("error!");
    else
        printf(" semaphore %s was opened.\n",semname);


    while(1)
    {
        sem_wait(exesem);
        if((counter%div)==0)
            printf("progress: %i\n",(int) counter/div);
        /* update the perception module */

        perc.read_sensor_from_shm();
        perc.update(mobl.GetMO());

        // update the motivational orientation
        // NOTE: A HELD BEHAVIOR WILL UPDATE THE MO ANYWAYS
        // GOOD OR BAD? THAT'S THE QUESTION.

        mobl.update(&r1,besh.GetBF(), perc.GetEO(),0.0/*RO*/);

        /* update the behavioral schemata */
        besh.update(perc.getEvents(),mobl.GetMO(),NO_BEHAVIOR,-1/*intending*/,modi2follow,obj2follow);

        // execute the current behavior
        if(c_beh!=NULL)
        {
            executor.update(atoi(c_beh));
            //printf("executing ");
            //prt_beh(atoi(c_beh));
        }
        else
            executor.update(besh.GetBehavior());
        //executor.update(0);
        /*
        if(besh.GetBehavior()==MODI_FOLLOW)
        {
            printf("here!%d\n",MODI_FOLLOW);
        }
        */
        if(csv_make)
        {
            prt_all(csv_file,r1,perc.GetEO(),perc.getEvents(),besh.GetBehavior());
        }
        else
        {
            prt_ModiSen(r1);
            //prt_beh(besh.GetBehavior());
        }
        executor.upd_motor_and_mo_to_shm(besh.GetBF());
        //usleep(20000);
        if(limit == counter)
            break;
        else
            counter++;
    }
    fclose(csv_file);
    // which is the selected behavior?
    cout << endl << "Done!" << endl;
    return 0;
}


