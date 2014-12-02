#include "../include/perception.h"

perception::perception()
{
    // no events detected
    for(int i=0; i<MAX_EVENTS; i++) events_act[i] = 0;
    EO = 0;
}

perception::perception(ModiSen* robot_inputs)
{
    inref = robot_inputs;
    // no events detected
    for(int i=0; i<MAX_EVENTS; i++) events_act[i] = 0;
    std::cout << "from class, ID: "<< inref->id<<std::endl;
    EO = 0;
}

perception::~perception()
{
    //dtor
}

void perception::reset(int low, int high){
    for(int i=low; i< high; i++)
    {
        events_act[i] = 0;
    }
}

int* perception::getEvents()
{
    return events_act;
};

void perception::initShm(int idx)
{
    key_t shkey1 = SH_SENSORS+idx;
    int shmid1;
    if ((shmid1 = shmget(shkey1, sizeof(ModiSen), IPC_CREAT | 0666)) < 0)
    {
        perror("shmget");
        exit(1);
    }
    if ((shref = (ModiSen*) shmat(shmid1, NULL, 0)) == (ModiSen *) -1)
    {
        perror("shmat");
        exit(1);
    }
}

void perception::read_sensor_from_shm()
{
    if(shref!=NULL) memcpy(inref,shref,sizeof(ModiSen));
}

void perception::update(float MO)
{
    for(int i = 0; i< MAX_EVENTS; i++)
        events_act[i] = 0;
    mo_bias = MO;
    int* csen = inref->colsen;
    /* some thresholds PLEASE CHECK!!!!! */
    float THR_APCEN = 0.15;     /*threshold approach center */
    float THR_AVCEN = 0.20;     /*threshold avoidance center */
    float THR_APLAT = 0.15;     /*threshold approach lateral */
    float THR_AVLAT = 0.20;     /*threshold avoidance lateral */
    float NEAR_APOBJD = 0.80;    /*threshold approach object near */
    float TOUC_APOBJD = 0.08;     /*threshold approach object touched */
    float NEAR_AVOBJD = 0.80;    /*threshold avoidance object near */
    float TOUC_AVOBJD = 0.12;     /*threshold avoidance object touched */

    /* APPROACH BEHAVIOR */
    if(mo_bias>0)
    {
        /* collisions */
        if(csen[7]||csen[0]||csen[1]) events_act[COL_FRONT] = 1;
        if(csen[3]||csen[4]||csen[5]) events_act[COL_BACK] = 1;
        if(csen[5]||csen[6]||csen[7]) events_act[COL_LEFT] = 1;
        if(csen[1]||csen[2]||csen[3]) events_act[COL_RIGHT] = 1;
        /* near to obstacles */
        if(inref->senc < THR_APCEN) events_act[NEAR_CENTER] = 1;
        if(inref->senl < THR_APLAT) events_act[NEAR_LEFT] = 1;
        if(inref->senr < THR_APLAT) events_act[NEAR_RIGHT] = 1;
        /* corners detections */
        if(inref->senc < THR_APCEN && inref->senl < THR_APLAT) events_act[CORNER_LEFT] = 1;
        if(inref->senc < THR_APCEN && inref->senr < THR_APLAT) events_act[CORNER_RIGHT] = 1;
        /* object detection or touch */
        for(int i=0; i< MAX_OBJECTS; i++)
        {
            if((inref->o_pos[i].m<NEAR_APOBJD)&&(inref->o_pos[i].m>=0))
            {
                if(inref->o_pos[i].m<TOUC_APOBJD) events_act[OBJ_TOUCH] = 1<<i;
                else events_act[OBJ_DETECT] = 1<<i;
            }
        }
        /* modies detection or touch */
        for(int i=0; i< MAX_MODIES; i++)
        {
            if((inref->m_pos[i].m<NEAR_APOBJD)&&(inref->m_pos[i].m>=0))
            {
                if(inref->m_pos[i].m<TOUC_APOBJD) events_act[MOD_TOUCH] = 1<<i;
                else events_act[MOD_DETECT] = 1<<i;
            }
        }
    }
    else /* AVOIDANCE BEHAVIOR */
    {
        /* collisions */
        if(csen[6]||csen[7]||csen[0]||csen[1]||csen[2]) events_act[COL_FRONT] = 1;
        if(csen[2]||csen[3]||csen[4]||csen[5]||csen[6]) events_act[COL_BACK] = 1;
        if(csen[4]||csen[5]||csen[6]||csen[7]||csen[0]) events_act[COL_LEFT] = 1;
        if(csen[0]||csen[1]||csen[2]||csen[3]||csen[4]) events_act[COL_RIGHT] = 1;
        /* near to obstacles */
        if(inref->senc < THR_AVCEN) events_act[NEAR_CENTER] = 1;
        if(inref->senl < THR_AVLAT) events_act[NEAR_LEFT] = 1;
        if(inref->senr < THR_AVLAT) events_act[NEAR_RIGHT] = 1;
        /* corners detections */
        if(inref->senc < THR_AVCEN && inref->senl < THR_AVLAT) events_act[CORNER_LEFT] = 1;
        if(inref->senc < THR_AVCEN && inref->senr < THR_AVLAT) events_act[CORNER_RIGHT] = 1;
        /* object detection or touch */
        for(int i=0; i< MAX_OBJECTS; i++)
        {
            if((inref->o_pos[i].m<NEAR_APOBJD)&&(inref->o_pos[i].m>=0))
            {
                if(inref->o_pos[i].m<TOUC_APOBJD) events_act[OBJ_TOUCH] = events_act[OBJ_TOUCH] + (1<<i);
                else events_act[OBJ_DETECT] = events_act[OBJ_DETECT] + (1<<i);
            }
        }
        /* modies detection or touch */
        for(int i=0; i< MAX_MODIES; i++)
        {
            if((inref->m_pos[i].m<NEAR_AVOBJD)&&(inref->m_pos[i].m>=0))
            {
                if(inref->m_pos[i].m<TOUC_AVOBJD) events_act[MOD_TOUCH] = events_act[MOD_TOUCH] + (1<<i);
                else events_act[MOD_DETECT] = events_act[MOD_DETECT] + (1<<i);
            }
        }
    }
    events_act[NULL_EVENT] = 1;
    for(int i=1; i< MAX_EVENTS; i++)
        {
            if(events_act[i])
            {
              events_act[NULL_EVENT] = 0;
              break;
            }
        }
}
