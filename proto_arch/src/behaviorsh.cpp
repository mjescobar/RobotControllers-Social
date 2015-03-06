#include "../include/behaviorsh.h"
#include <stdio.h>

void p_beh(int idx)
{
    std::string str_beh[] = {
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
    if(idx==-1) printf("NO_BEHAVIOR\n");
    else printf("%s\n",str_beh[idx].c_str());
}

void behaviorsh::load_priorities()
{
    for(int i = 0; i < MAX_BEHAVIORS; i++) behaviorsPR[i] = behaviorsPR_def[i];
}

void behaviorsh::act(int where, float factor, int val)
{
    behaviors[where] = val;
    behaviorsPR[where] = factor*behaviorsPR_def[where];
}

void behaviorsh::normalize()
{
    float sum = 0;
    for(int i=0; i < MAX_BEHAVIORS; i++) sum = (behaviors[i])? sum + behaviorsPR[i] : sum;
    for(int i=0; i < MAX_BEHAVIORS; i++) behaviorsPR[i] = (behaviors[i])? behaviorsPR[i]/sum : 0;
}

float minDistRang(float val)
{
    float dm1 = absv(val+1);
    float dp1 = absv(val-1);
    float d0 = absv(val);
    float out = ((dm1<dp1) && (dm1 < d0))? -1:
                ((dp1<dm1) && (dp1 < d0))?  1: 0;
    return out;
}

/// hacer reingenieria para las probabilidades de comportamiento
int behaviorsh::resolveCollisions()
{
    distribution(generator);
    float val = distribution(generator);
    float sum = 0;
    int selected = -1;
    for(int i=0; i < MAX_BEHAVIORS; i++)
    {
        // printf("%d",behaviors[i]);
        sum = sum + behaviorsPR[i];
        if(sum>=val)
        {
            selected = i;
            break;
        }
    }
    // printf("\n%f, %d\n",sum, selected);
    for(int i=0; i < MAX_BEHAVIORS; i++)
    {
        if(i==selected) behaviors[i] = 1;
        else behaviors[i] = 0;
    }
    return selected;
}

behaviorsh::behaviorsh()
{
    //ctor
    for(int i=0; i < MAX_BEHAVIORS; i++)
    {
        behaviors[i] = 0;
        behaviorsMO[i] = 0.0;
        behaviorsPR[i] = 0.0;
        behaviorsPR_def[i] = 0.0;
    }
    load_orientations();
    behaviorsPR_def[FULL_STOP]      = 1;
    behaviorsPR_def[WALL_FOLLOW]    = 1;
    behaviorsPR_def[EVADE_OBJ]      = 1;
    behaviorsPR_def[REACH_OBJ]      = 1;
    behaviorsPR_def[FREE_EXP]       = 1;
    behaviorsPR_def[MARK_CORNERS]   = 0.0;
    behaviorsPR_def[TURN_180]       = 5;
    behaviorsPR_def[MODI_FOLLOW]    = 1;
    behaviorsPR_def[MORE_SPD]       = 1;
    behaviorsPR_def[LESS_SPD]       = 1;
    behaviorsPR_def[EVADE_OBST]     = 10;
    distribution = std::uniform_real_distribution<float>(0.0,1.0);

    sel_behavior = 0;
}

behaviorsh::~behaviorsh()
{
    //dtor
}

void behaviorsh::mo_bias(float MO)
{
    float f1 = 1.2; // if behavior and current MO are compatible
    float f2 = 0.8; // if behavior and current MO are NOT compatible
    for(int i=0; i < MAX_BEHAVIORS; i++)
    {
        if((behaviorsMO[i]*MO)>=0)  behaviorsPR[i] = f1*behaviorsPR[i];
        else behaviorsPR[i] = f2*behaviorsPR[i];
    }

}

void behaviorsh::load_orientations()
{
    behaviorsMO[FULL_STOP]      = 0;
    behaviorsMO[MARK_CORNERS]   = 0;
//
//    behaviorsMO[WALL_FOLLOW]    = 0.5*AV;
//    behaviorsMO[EVADE_OBJ]      = AV;
//    behaviorsMO[TURN_180]       = AV;
//    behaviorsMO[LESS_SPD]       = 0.25*AV;
//    behaviorsMO[EVADE_OBST]     = AV;
//
//    behaviorsMO[REACH_OBJ]      = AP;
//    behaviorsMO[FREE_EXP]       = 0.5*AP;
//    behaviorsMO[MODI_FOLLOW]    = AP;
//    behaviorsMO[MORE_SPD]       = 0.25*AP;
//

    behaviorsMO[WALL_FOLLOW]    = AV;
    behaviorsMO[EVADE_OBJ]      = AV;
    behaviorsMO[TURN_180]       = AV;
    behaviorsMO[LESS_SPD]       = AV;
    behaviorsMO[EVADE_OBST]     = AV;

    behaviorsMO[REACH_OBJ]      = AP;
    behaviorsMO[FREE_EXP]       = AP;
    behaviorsMO[MORE_SPD]       = AP;
//  test it! originally, AP
    behaviorsMO[MODI_FOLLOW]    = AP;
}


void behaviorsh::update(int *e, float MO, int hold_behavior, int reasonedb, int modi2follow, int obj2follow)
{
    load_priorities();
    mo_bias(MO);
/*
#define NULL_EVENT      #define COL_FRONT       #define COL_BACK        #define COL_LEFT
#define COL_RIGHT       #define NEAR_LEFT       #define NEAR_CENTER     #define NEAR_RIGHT
#define CORNER_LEFT     #define CORNER_RIGHT    #define OBJ_DETECT      #define MOD_DETECT
#define OBJ_TOUCH       #define MOD_TOUCH

#define FULL_STOP       #define WALL_FOLLOW     #define REACH_OBJ       #define EVADE_OBJ
#define FREE_EXP        #define MARK_CORNERS    #define TURN_180        #define MODI_FOLLOW
#define MORE_SPD        #define LESS_SPD        #define EVADE_OBST
*/
    if(MO>0) // AP
    {
        if(DEBUG) printf("approach used.\n");
        // no events detected.
        if(e[NULL_EVENT]) act(FREE_EXP,1.0);
        if(e[NULL_EVENT]) act(MORE_SPD,0.1);
        if(e[NULL_EVENT]) act(LESS_SPD,0.025);
        // evading collisions.
        if(e[COL_LEFT] || e[COL_FRONT] || e[COL_BACK] || e[COL_RIGHT]) act(EVADE_OBST,1.0);
        // near obstacles.
        if(e[NEAR_CENTER] || e[NEAR_LEFT] || e[NEAR_RIGHT]) act(LESS_SPD,0.1);
        if(e[NEAR_CENTER] || e[NEAR_LEFT] || e[NEAR_RIGHT]) act(FREE_EXP,1.0);
        // a dead end way.
        if(e[CORNER_LEFT] && e[CORNER_RIGHT]) act(TURN_180,1.0);
        // corners detection.
        if(e[CORNER_LEFT] || e[CORNER_RIGHT]) act(MARK_CORNERS,1.0);
        if(e[CORNER_LEFT] || e[CORNER_RIGHT]) act(LESS_SPD,1.0);
        // any object detected.
        if(e[OBJ_DETECT]) act(FREE_EXP,1.0);
        if(e[OBJ_DETECT]) act(REACH_OBJ,0.1);
        // if the robot should follow only the obj zero, and evade the rest
        if(obj2follow==ALL_BUT_ZERO)
        {
            if(e[OBJ_DETECT]&(1<<obj2follow)) act(REACH_OBJ,1.0);
            else act(EVADE_OBJ,1.0);
        }
        // when the detected object is the right one.
        if(obj2follow>=0)
        {
            if(e[OBJ_DETECT]&(1<<obj2follow)) act(REACH_OBJ,1.0);
        }
        // when any modi is detected.
        if(e[MOD_DETECT]) act(FREE_EXP,1.0);
        if(e[MOD_DETECT]) act(MORE_SPD,0.2);
        if(e[MOD_DETECT]) act(LESS_SPD,0.1);
        // by default, the robots will follow to any modi
        if(e[MOD_DETECT]&&(modi2follow!=NO_MODI)) act(MODI_FOLLOW,0.15);
        // when the detected modi is the right one.
        if(e[MOD_DETECT]&&(modi2follow>=0))
        {
            if(e[MOD_DETECT]&(1<<modi2follow))
            {
                act(MODI_FOLLOW,0.5);
            }
        }
    }
    else
    {
        if(DEBUG) printf("avoidance used.\n");
        // no events detected.
        if(e[NULL_EVENT]) act(WALL_FOLLOW,1.0);
        if(e[NULL_EVENT]) act(LESS_SPD,0.1);
        if(e[NULL_EVENT]) act(MORE_SPD,0.025);
        // evading collisions.
        if(e[COL_LEFT] || e[COL_FRONT] || e[COL_BACK] || e[COL_RIGHT]) act(EVADE_OBST,1.0);
        // near obstacles.
        if(e[NEAR_CENTER] || e[NEAR_LEFT] || e[NEAR_RIGHT]) act(LESS_SPD,0.1);
        if(e[NEAR_CENTER] || e[NEAR_LEFT] || e[NEAR_RIGHT]) act(WALL_FOLLOW,1.0);
        // a dead end way.
        if(e[CORNER_LEFT] && e[CORNER_RIGHT]) act(TURN_180,1.0);
        // corners detection.
        if(e[CORNER_LEFT] || e[CORNER_RIGHT]) act(MARK_CORNERS,1.0);
        if(e[CORNER_LEFT] || e[CORNER_RIGHT]) act(LESS_SPD,1.0);
        // any object detected.
        if(e[OBJ_DETECT]) act(EVADE_OBJ,1.0);
        if(e[OBJ_DETECT]) act(WALL_FOLLOW,1.0);
        // if the robot should follow only the obj zero, and evade the rest
        if(obj2follow==ALL_BUT_ZERO)
        {
            if(e[OBJ_DETECT]&(1<<obj2follow)) act(REACH_OBJ,1.0);
            else act(EVADE_OBJ,1.0);
        }
        // when the detected object is the right one.
        if(obj2follow>=0)
        {
            if(e[OBJ_DETECT]&(1<<obj2follow)) act(REACH_OBJ,1.0);
        }
        // when any modi is detected.
        if(e[MOD_DETECT]&&(modi2follow!=NO_MODI)) act(MODI_FOLLOW,0.3);
        if(e[MOD_DETECT]) act(MORE_SPD,0.2);
        if(e[MOD_DETECT]) act(FREE_EXP,0.1);
        /*
        if(e[MOD_DETECT]&&(modi2follow!=NO_MODI)) act(MODI_FOLLOW,0.8);
        if(e[MOD_DETECT]) act(MORE_SPD,0.4);
        if(e[MOD_DETECT]) act(FREE_EXP,0.4);
        */
        if(e[MOD_DETECT]) act(WALL_FOLLOW,1.0);
        // when the detected modi is the right one.
        if(e[MOD_DETECT]&&(modi2follow>=0))
        {
            if(e[MOD_DETECT]&(1<<modi2follow))
            {
                act(MODI_FOLLOW,2.0);
            }
        }
    }
    /*  NOTE: THIS NORMALIZATION FUNCTION ONLY CONSIDER THE SELECTED
        BEHAVIORS. THAT MEANS THAT ALL THE OTHER PRIORITIES GOES TO ZERO.
    */
    normalize();
    int impulsive_behavior = resolveCollisions();
    int reflexive_behavior = reasonedb;

    // right now it only replaces the behavior
    if(reflexive_behavior!=NO_BEHAVIOR) sel_behavior = reflexive_behavior;
    else sel_behavior = impulsive_behavior;
    // if there is any behavior
    if(DEBUG)
    {
        printf("\t\tsel_behavior:\n\t\t");
        p_beh(sel_behavior);
    }
    sel_behavior = (hold_behavior==NO_BEHAVIOR)? sel_behavior : hold_behavior;
}

float behaviorsh::GetBF()
{
    return behaviorsMO[sel_behavior];
}
