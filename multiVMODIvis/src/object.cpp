#include "object.h"

object::object()
{
    data.id = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.pos.a = 0;
    data.flag = 0;

    patt_id = 0;
    data.id = 0;
    k_index = 0;
    cf = 0;
}


object::object(int inpatt_id)
{
    data.id = 0;
    data.pos.x = 0;
    data.pos.y = 0;
    data.pos.a = 0;
    data.flag = 0;

    patt_id = inpatt_id;
    data.id = patt_id;
    k_index = 0;
    cf = 0;
}

bool object::isVisible()
{
    return (k_index>-1);
}

int object::getKidx()
{
    return k_index;
}
void object::setKidx(int value)
{
    k_index = value;
}

// unfiltered
void object::updateAngle(double angval)
{
    data.pos.a = angval;
}

void object::setpos(int inx, int iny,double incf)
{
    data.pos.x = inx;
    data.pos.y = iny;
    cf = incf;
}

void *obj_connection(void *arguments)
{
    object* lobj = (object*) arguments;
   // flushing the console
    fflush(stdout);
    ///let the magic begin!
    int lsockid = lobj->getSocket();
    float outdata[3];
    outdata[0] = lobj->getpos().x/(PPCMX*100);
    outdata[1] = lobj->getpos().y/(PPCMY*100);
    outdata[2] = lobj->getpos().a;
    replyToReceivedData((char*)outdata,sizeof(float)*3, lobj->getSocket());
    while(!shutdown_flag)
    {
//        lobj->swait();
        int receivedDataLength;
        char* receivedData=receiveData(receivedDataLength,lsockid);
        if (receivedData!=NULL)
        {
            // We received data. The server ALWAYS replies!
            // filling the allmodi_data array
            //lmodi->update_act(((float*)receivedData)[0],-1*((float*)receivedData)[1],((float*)receivedData)[2]);
            // if(myid==0) simulation_time = ((float*)receivedData)[3];
            delete[] receivedData;
            outdata[0] = lobj->getpos().x/(PPCMX*100)-1.18;
            outdata[1] = 0.68-lobj->getpos().y/(PPCMY*100);
            outdata[2] = lobj->getpos().a;
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
    printf("obj %d disconnected.\n",lobj->getPattid());
    close(lsockid);
    return NULL;
}

pta object::getpos()
{
    return data.pos;
}

void object::setSocket(int i,object *ref)
{
    printf("socket %d attached to object %d.\n",i,patt_id);
    sockid = i;
    thrd = pthread_create(&tid,NULL,obj_connection,ref);
    pthread_detach(tid);
}

int object::getPattid()
{
    return patt_id;
}

int object::getSocket()
{
    return sockid;
}


object::~object()
{
    //dtor
}


