#include "../include/behexec.h"
#include <stdio.h>

float clip(float val)
{
    float out = val;
    if(val>=MAX_SPD) out=MAX_SPD;
    if(val<0) out = 0;
    return out;
}

float get_dist(float current, float target, int dir)
{
    if(dir==CW)
    {
        if(current<target)
            return target-current;
        else
        {
            float d1 = 180-current;
            float d2 = 180+target;
            return d1+d2;
        }
    }
    else
    {
        if(current>target)
            return current-target;
        else
        {
            float d1 = 180-target;
            float d2 = 180+current;
            return d1+d2;
        }
    }
}

float valid_angle(float ang)
{
    return (ang>180)? ang-360:(ang<-180)? 360+ang:ang;
}

void behexec::initShm(int idx)
{
    key_t shkey2 = SH_MOTORS+idx;
    int shmid2;
    if ((shmid2 = shmget(shkey2, sizeof(motor_and_mo), IPC_CREAT | 0666)) < 0)
    {
        printf(" error in modi %d pos %d, get motor sh ",idx,shkey2);
        perror("shmget");
        exit(1);
    }
    if ((motmosh = (motor_and_mo*) shmat(shmid2, NULL, 0)) == (motor_and_mo*) -1)
    {
        perror("shmat");
        exit(1);
    }

}

void behexec::upd_motor_and_mo_to_shm(float mo)
{
    motor tmp;
    tmp.left = 0.0;
    tmp.right = 0.0;
    float inmo = mo;
    if(motmosh!=NULL)
    {
        memcpy(&(motmosh->mot),&mot,sizeof(motor));
        //memcpy(&(motmosh->mot),&tmp,sizeof(motor));
        memcpy(&(motmosh->mo),&inmo,sizeof(float));
    }
}

void behexec::f_full_stop()
{
    // just stop the things
    setVel(0,0);
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_free_exp()
{
    // retrieve current values of the robot's sensors
    float cs = rob->senc;
    float ls = rob->senl;
    float rs = rob->senr;

    // read the pre-set speed of the motors
    int cState = s_free_exp.cState;
    float minNoObj = s_free_exp.minNoObj;
    float minObjLat = s_free_exp.minObjLat;
    float critObjLat = s_free_exp.critObjLat;
    float hNorm = s_free_exp.hist_norm;
    float hCrit = s_free_exp.hist_crit;
    // main finite-state-machine
    switch(cState)
    {
    case 1: //forward
        setVel(default_speed,-default_speed);
        if(((cs<minNoObj)&&(rs<ls)) || rs<(minObjLat)) cState = 2; //go left
        if(((cs<minNoObj)&&(ls<rs)) || ls<(minObjLat)) cState = 3; //go right
        break;
    case 2: // left
        setVel(0.0,-0.5*default_speed);
        if(rs < critObjLat)  cState = 4; // critical turn left
        if(cs > minNoObj+hNorm) cState = 1; // back to forward
        break;
    case 3: // right
        setVel(0.5*default_speed,0.0);
        if(ls < critObjLat)  cState = 5; // critical turn right
        if(cs > minNoObj+hNorm) cState = 1; // back to forward
        break;
    case 4: // critical left
        setVel((-0.5)*default_speed,-0.5*default_speed);
        if(rs > critObjLat+hCrit)  cState = 2; // normal turn left
        if(cs > minNoObj+hCrit) cState = 1; // back to forward
        break;
    case 5: // critical right
        setVel(0.5*default_speed,0.5*default_speed);
        if(ls > critObjLat+hCrit)  cState = 3; // normal turn right
        if(cs > minNoObj+hCrit) cState = 1; // back to forward
        break;
    }
    s_free_exp.cState = cState;
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_wall_follow()
{
    float sc = rob->senc;
    float sl = rob->senl;
    float sr = rob->senr;
//    int dir = s_wall_follow.direction; /* CW right, CCW left */
    float cdist = s_wall_follow.close_dist;
    float critic = s_wall_follow.crit_dist;
    float pdist = s_wall_follow.prev_dist;
    if(prev_behavior!=WALL_FOLLOW)
        s_wall_follow.recover_wall = 0;

    /*  positive means that we are increasing the distance to the wall.
        negative means that we are decreasing the distance to the wall. */

    float delta = sr-pdist;

    /*  positive means less distance than WALL_FOLLOW_DIST
        negative means more distance than WALL_FOLLOW_DIST
        zero means that we are in the WALL_FOLLOW_DIST!    */
    float factor = (WALL_FOLLOW_DIST-sr)/(2*WALL_FOLLOW_DIST);

    if((sc>=cdist)&&(sr> cdist))
    {
        if(delta>0||s_wall_follow.recover_wall)
        {
            s_wall_follow.recover_wall = 1;
            setVel(default_speed,-0.3*default_speed);
            if(delta<0)
                s_wall_follow.recover_wall = 0;
        }
        else
            setVel(default_speed,-default_speed);
    }
    if((sc>=cdist)&&(sr<=cdist)&&(sr< 2*WALL_FOLLOW_DIST))
        setVel((1-factor)*default_speed,-(1+factor)*default_speed);

    if((sc>=cdist)&&(sr<=cdist)&&(sr>=2*WALL_FOLLOW_DIST))
    {
        if(delta>0||s_wall_follow.recover_wall)
        {
            s_wall_follow.recover_wall = 1;
            setVel(default_speed,-0.3*default_speed);
            if(delta<0)
                s_wall_follow.recover_wall = 0;
        }
        else
            setVel(default_speed,-0.6*default_speed);
    }

    if((sc< cdist)&&(sr> sc))
        setVel(0.6*default_speed,-default_speed);

    if((sc< cdist)&&(sr<=sc)&&(sr< 2*WALL_FOLLOW_DIST))
        setVel((1-factor)*default_speed,-(1+factor)*default_speed);

    if((sc< cdist)&&(sr<=sc)&&(sr>=2*WALL_FOLLOW_DIST))
    {
        if(delta>0||s_wall_follow.recover_wall)
        {
            s_wall_follow.recover_wall = 1;
            setVel(default_speed,-0.3*default_speed);
            if(delta<0)
                s_wall_follow.recover_wall = 0;
        }
        else
            setVel(default_speed,-0.6*default_speed);
    }

    if(sc<=critic||sl<=critic||sr<=critic)
    {
        if(sl<=critic)
            setVel(0.5*default_speed,0.5*default_speed);
        else if(sr<=critic)
            setVel(-0.5*default_speed,-0.5*default_speed);
        else
        {
            if(sl<sr)
                setVel(0.5*default_speed,0.5*default_speed);
            else
                setVel(-0.5*default_speed,-0.5*default_speed);
        }
    }
    s_wall_follow.prev_dist = sr;
}

void behexec::f_more_spd()
{
    default_speed = clip(default_speed + s_more_spd.delta);
    setVel(default_speed, -default_speed);
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_less_spd()
{
    default_speed = clip(default_speed - s_more_spd.delta);
    if(default_speed<MIN_SPD) default_speed = MIN_SPD;
    setVel(default_speed, -default_speed);
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_turn_180()
{
    //default_speed = DEF_SPD;
    int cState = s_turn_180.cState;
    float prevAng = s_turn_180.prev_angle;
    float currentAng = rob->pos.a;
    float targetAng = s_turn_180.target_angle;
    float sTurn = s_turn_180.sturn;
    switch(cState)
    {
    case 0: // just started!
        hold_behavior = TURN_180;
        s_turn_180.start_angle = rob->pos.a;
        s_turn_180.prev_angle = rob->pos.a;
        ///
        s_turn_180.current_speed = MIN_SPD;// default_speed;
        s_turn_180.target_angle = (rob->pos.a >= 0)? (rob->pos.a- 180) : (rob->pos.a + 180);
        s_turn_180.cState = 1;
        s_turn_180.sturn = CW;
        break;
    case 1: // approaching to the objective, linear desaceleration
        if(get_dist(currentAng,targetAng,sTurn)<=ANGLE_ERROR)
        {
            setVel(0,0);
            s_turn_180.cState = 2;
        }
        else
        {
            if(get_dist(currentAng,targetAng,sTurn)>get_dist(prevAng,targetAng,sTurn))
            s_turn_180.sturn = sTurn = (sTurn==CW)? CCW:CW;

            float base = 30;
            float factor = (base + get_dist(currentAng,targetAng,sTurn))/180;
            factor = factor>1? 1: factor;
            if(sTurn==CW)   // clockwise turn (CW)
                setVel(factor*default_speed,factor*default_speed);
            else            // counterclockwise turn (CCW)
                setVel((-1)*factor*default_speed,factor*default_speed);
        }
        s_turn_180.prev_angle = currentAng;
        break;
    case 2:
        s_turn_180.cState = 0;
        s_turn_180.sturn = CW;
        hold_behavior = NO_BEHAVIOR;
        setVel(0,0);
    }
}

void behexec::f_reach_obj()
{
    int cState = s_reach_obj.cState;
    int obj2detect = s_reach_obj.obj2detect;
    if(obj2detect==NO_OBJ)
        return;
    if(obj2detect==ALL_BUT_ZERO)
        obj2detect = 0;
    float target_error, distance, speed, error_fact;
    float mindist = 2*FAR_DIST;
    switch(cState)
    {
    case 0:
        s_reach_obj.original_spd = default_speed;
        cState = 2;
        break;
    case 1:
        // currently, there's no case 1.
        break;
    case 2:
        /* alignment */
        if(obj2detect==ANY_OBJ)
        {
            for(int i=0; i< MAX_OBJECTS; i++)
                if(rob->o_pos[i].m>=0)
                {
                    if(rob->o_pos[i].m<=mindist)
                    {
                        mindist = rob->o_pos[i].m;
                        obj2detect = i;
                    }
                }
        s_reach_obj.original_spd = default_speed;
        }
        if(obj2detect==ANY_MODI)
        {
            setVel(0.0,0.0);
            break;
        };
        target_error = rob->o_pos[obj2detect].a;
        distance = rob->o_pos[obj2detect].m;
        speed = s_reach_obj.original_spd;
        //printf("%f\n",target_error);
        /* magnitude according to the distance */
        speed = (speed <= MIN_SPD)? MIN_SPD : (0.3+distance)*speed;
        error_fact = 1.5;
        if(target_error>(error_fact*ANGLE_ERROR))
            setVel((speed*(1+target_error/180.0)),(-1.0)*(speed*(1-2*target_error/180.0)));
        else if(target_error<(-1*error_fact*ANGLE_ERROR))
            setVel((speed*(1+2*target_error/180.0)),(-1.0)*(speed*(1-target_error/180.0)));
        else
            setVel((speed),(-1.0)*(speed));
        if(distance <= COL_DIST)
            setVel(0,0);
        break;
    }
    s_reach_obj.cState = cState;
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_evade_obj()
{
    int cState = s_evade_obj.cState;
    int obj2detect = s_evade_obj.obj2detect;
    if(obj2detect==NO_OBJ)
        return;
    float target_error, distance, speed, error_fact;
    int ii = 0;
    float mindist = 2*FAR_DIST;
    switch(cState)
    {
    case 0:
        s_evade_obj.original_spd = default_speed;
        cState = 2;
        break;
    case 1:
        // currently, there's no case 1.
        break;
    case 2:
            /* alignment */
        if(obj2detect==ANY_OBJ||obj2detect==ALL_BUT_ZERO)
        {
            if(obj2detect==ALL_BUT_ZERO) ii = 1;
            for(int i=ii; i< MAX_OBJECTS; i++)
                if(rob->o_pos[i].m>=0)
                {
                    if(rob->o_pos[i].m<=mindist)
                    {
                        mindist = rob->o_pos[i].m;
                        obj2detect = i;
                    }
                }
        s_evade_obj.original_spd = default_speed;
        }
        if(obj2detect==ANY_MODI)
        {
            setVel(0.0,0.0);
            break;
        };
        target_error = valid_angle(rob->o_pos[obj2detect].a+180.0);
        distance = rob->o_pos[obj2detect].m;
        if((distance <0)||(distance >= FAR_DIST))
        {
            setVel(0,0);
            break;
        }
        speed = s_evade_obj.original_spd;
        /* magnitude according to the distance */
        speed = MAX_SPD;
        error_fact = 4;
        if(target_error>(error_fact*ANGLE_ERROR))
            setVel((speed*(1+target_error/180.0)),(-1.0)*(speed*(1-2*target_error/180.0)));
        else if(target_error<(-1*error_fact*ANGLE_ERROR))
            setVel((speed*(1+2*target_error/180.0)),(-1.0)*(speed*(1-target_error/180.0)));
        else
        {
            if(rob->senc>NEAR_DIST)
                setVel((speed),(-1.0)*(speed));
            else
            {
                if(rob->senl>rob->senr)
                    setVel(0.8*(speed),(-1.0)*(speed));
                else
                    setVel((speed),0.8*(-1.0)*(speed));
            }
        }
        break;
    }
    s_evade_obj.cState = cState;
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_modi_follow()
{
    int cState = s_modi_follow.cState;
    int modi2detect = s_modi_follow.modi2detect;
    if(modi2detect==NO_MODI)
        return;
    float prev_dist = s_modi_follow.prev_dist;
    float distance = -1;
    float target_error, speed, error_fact;
    float mindist = 2*FAR_DIST;
    switch(cState)
    {
    case 0:
        s_modi_follow.original_spd = default_speed;
        cState = 2;
        break;
    case 1:
        // currently, there's no case 1.
        break;
    case 2:
        /* alignment */
        if(modi2detect==ANY_MODI)
        {
            for(int i=0; i< MAX_OBJECTS; i++)
                if((rob->m_pos[i].m>=0)&&i!=rob->id)
                {
                    if(rob->m_pos[i].m<=mindist)
                    {
                        mindist = rob->m_pos[i].m;
                        modi2detect = i;
                    }
                }
        s_modi_follow.original_spd = default_speed;
        }
        if(modi2detect==ANY_MODI) modi2detect = rob->id;
        target_error = rob->m_pos[modi2detect].a;
        distance = rob->m_pos[modi2detect].m;
        if(distance <0)
        {
            setVel(0.0,0.0);
            break;
        }
        speed = s_reach_obj.original_spd;
        //printf("\tmfol: %d, %f,%f\n",modi2detect,target_error,distance);
        /* magnitude according to the distance */
        speed = 10*(distance-MODI_FOLLOW_DIST)*default_speed+MIN_SPD;
        speed = speed > MAX_SPD ? MAX_SPD : speed;
        //printf("dist: %f,spd: %f\n",distance, speed);
        error_fact = 1.5;
        if(target_error>(error_fact*ANGLE_ERROR))
            setVel((speed*(1+target_error/180.0)),(-1.0)*(speed*(1-2*target_error/180.0)));
        else if(target_error<(-1*error_fact*ANGLE_ERROR))
            setVel((speed*(1+2*target_error/180.0)),(-1.0)*(speed*(1-target_error/180.0)));
        else
            setVel((speed),-1.0*(speed));
        break;
        if(distance<=MODI_FOLLOW_DIST)
        {
            s_modi_follow.prev_dist = distance;
            cState = 3;
            break;
        }
    case 3:
        setVel(0,0);
        if((distance-prev_dist)>MODI_FOLLOW_DIST)
            cState = 2;
        break;
    }

    s_modi_follow.cState = cState;
    hold_behavior = NO_BEHAVIOR;
}

void behexec::f_evade_obst()
{
    /* for now, it will evade in the direction with more space */
    hold_behavior = EVADE_OBST;
    float sl = rob->senl;
    float sc = rob->senc;
    float sr = rob->senr;
    float sum_all_num,sum_all_den;
    float sum_front_num, sum_front_den;
    float sum_back_num, sum_back_den;
    sum_all_num   = sum_all_den   = 0;
    sum_front_num = sum_front_den = 0;
    sum_back_num  = sum_back_den  = 0;
    float current_angle = 0;
    for(int i=0; i<NCSENS; i++)
    {
        current_angle = valid_angle(i*rob->colsen[i]*360.0/NCSENS);
        sum_all_num = sum_all_num + current_angle;
        sum_all_den = sum_all_den + rob->colsen[i];
        if((current_angle>=-90)&&(current_angle<=90))
        {
            sum_front_num = sum_front_num + current_angle;
            sum_front_den = sum_front_den + rob->colsen[i];
        }
        else
        {
            sum_back_num = sum_back_num + i*rob->colsen[i]*360.0/NCSENS;
            sum_back_den = sum_back_den + rob->colsen[i];
        }
    }
    //printf("all %f, back %f,front %f\n",sum_all_den,sum_back_den,sum_front_den);
    if(sum_all_den == 0)
    {
        /* yay! no obstacles! */
        setVel(default_speed,-default_speed);
        hold_behavior = NO_BEHAVIOR;
        return;
    }
//    float mean_all = sum_all_num/sum_all_den;
    //printf("\nall: %f ",mean_all);
    if(sum_front_den == 0)
    {
        /* no obstacles in the front */
        if(sc>=sl||sc>=sr)  /* all the front is clear! -- I guess */
            setVel(default_speed,-default_speed);
        else if((sl>=sc)&&(sl>=sr))     /* more space in the left */
            setVel(0.5*default_speed,-default_speed);
        else if((sr>=sc)&&(sr>=sl))     /* more space in the right */
            setVel(default_speed,-0.5*default_speed);
        return;
    }
    float mean_front = sum_front_num/sum_front_den;
    //printf("front: %f ",mean_front);
    float esc_front = valid_angle(180.0+mean_front);
    if(sum_back_den == 0)
    {
        /* no objects in the back! perfect to turn wherever I want */
        if(esc_front<0)
        {
            /* turn left backwards */
            setVel(-default_speed,0.3*default_speed);
        }
        else
        {
            /* turn right backwards */
            setVel(-0.3*default_speed,default_speed);
        }
        return;
    }
    /* if nothing of the previous situations happened, then it's
       time to try to turn in my own position :3                   */
    float mean_back = valid_angle(sum_back_num/sum_back_den);
    //printf("back: %f ",mean_back);
    float esc_back = valid_angle(180.0+mean_back);
    if(esc_front*esc_back>0.0)
    {
        /* same sign */
        if(esc_front>0)
        {
            /* turn right   */
            setVel(default_speed,default_speed);
        }
        else
        {
            /* turn left    */
            setVel(-default_speed,-default_speed);
        }
    }
    else
    {
        /* different sign! */
        float error = esc_back+esc_front;
        if(error<-ANGLE_ERROR)          /* align with free space on its left */
            setVel(-default_speed,-default_speed);
        else if(error>ANGLE_ERROR)      /* align with free space on its right */
            setVel(default_speed,default_speed);
        else                            /* force movement >.< */
            setVel(default_speed,-default_speed);
    }
    return;
}

int behexec::GetLM()
{
    return clip(mot.left);
}

int behexec::GetRM()
{
    return clip(mot.right);
}

behexec::behexec()
{
   reset(ANY_MODI,ANY_OBJ);
}

void behexec::reset(int modi2detect, int obj2detect)
{
    //printf("starting!\n");
    hold_behavior = NO_BEHAVIOR;
    current_behavior = NO_BEHAVIOR;
    default_speed = DEF_SPD;

    mot.left = default_speed;
    mot.right = default_speed;

    // s_free_exp structure
    /*
    s_free_exp.minNoObj = 0.25;
    s_free_exp.minObjLat = 0.25;
    s_free_exp.critObjLat = 0.20;
    s_free_exp.hist_norm = 0.5;
    s_free_exp.hist_crit = 0.10;
    s_free_exp.cState = 1;
    */

    s_free_exp.minNoObj = 0.30;
    s_free_exp.minObjLat = 0.35;
    s_free_exp.critObjLat = 0.20;
    s_free_exp.hist_norm = 0.025;
    s_free_exp.hist_crit = 0.5;
    s_free_exp.cState = 1;

    // s_full_stop structure
    s_full_stop.prev_spd = 0;

    // more spd
    s_more_spd.delta = DEF_DELTA;

    // less spd
    s_less_spd.delta = DEF_DELTA;

    // turn 180
    s_turn_180.cState = 0;
    s_turn_180.sturn = CW;

    // wall following
    /*
    s_wall_follow.close_dist = 0.60;
    s_wall_follow.direction = CW;
    s_wall_follow.crit_dist = 0.15;
    s_wall_follow.prev_dist = 2*DEFAULT_RANGE;
    s_wall_follow.recover_wall = 0;
    */
    s_wall_follow.close_dist = 0.40;
    s_wall_follow.direction = CW;
    s_wall_follow.crit_dist = 0.25;
    s_wall_follow.prev_dist = 2*DEFAULT_RANGE;
    s_wall_follow.recover_wall = 0;



    /* reach object */
    s_reach_obj.cState = 0;
    s_reach_obj.obj2detect = obj2detect;
    s_reach_obj.original_spd = DEF_SPD;

    /* evade object */
    s_evade_obj.cState = 0;
    s_evade_obj.obj2detect = obj2detect;
    s_evade_obj.original_spd = DEF_SPD;

    /* modi follow */
    s_modi_follow.cState = 0;
    s_modi_follow.modi2detect = modi2detect;
    s_modi_follow.original_spd= DEF_SPD;
    s_modi_follow.prev_dist = -1;

    /* evade obstacle */
    s_evade_obst.cState = 0;

}

behexec::~behexec()
{
    //dtor
}

void behexec::setVel(float ml, float mr)
{
    if(ml!=KEEP_SPD) mot.left = ml;
    if(mr!=KEEP_SPD) mot.right = mr;

}

void behexec::update(int sel_behavior)
{
    current_behavior = (hold_behavior==NO_BEHAVIOR)? sel_behavior : hold_behavior;
    switch(current_behavior)
    {
    case FULL_STOP:
        f_full_stop();
        break;
    case FREE_EXP:
        f_free_exp();
        break;
    case MORE_SPD:
        f_more_spd();
        break;
    case LESS_SPD:
        f_less_spd();
        break;
    case TURN_180:
        f_turn_180();
        break;
    case WALL_FOLLOW:
        f_wall_follow();
        break;
    case REACH_OBJ:
        f_reach_obj();
        break;
    case EVADE_OBJ:
        f_evade_obj();
        break;
    case MODI_FOLLOW:
        f_modi_follow();
        break;
    case EVADE_OBST:
        f_evade_obst();
    case NO_BEHAVIOR:
    default:
        break;
    }
    prev_behavior = current_behavior;
}
