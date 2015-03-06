#include "smodi.h"
#include "math.h"
#include <stdlib.h>


#define FRM_FLAG 1
#define COM_FLAG 2


/*
    all the functions of this file are related with the smodi object.
    each name is self-explanatory.
*/

void setval(pta *pt, float x, float y, float a = 0.0)
{
    pt->x = x;
    pt->y = y;
    pt->a = a;
}

// constructors
smodi::smodi()
{
    data_to_control.id = -1;
}

float valid_angle(float ang)
{
    return (ang>180)? ang-360:(ang<-180)? 360+ang:ang;
}

void rotate_point(pta *out, pta src, pta pivot, float sinx, float cosx)
{
    out->x = (int)(cosx*(src.x-pivot.x)-sinx*(src.y-pivot.y)+pivot.x);
    out->y = (int)(sinx*(src.x-pivot.x)+cosx*(src.y-pivot.y)+pivot.y);
}

void smodi::upd_rotpx()
{
    float angle_c = data_to_control.pos.a;
    float angle_l = valid_angle(angle_c-ANGLE_BTW_PROXSENS);
    float angle_r = valid_angle(angle_c+ANGLE_BTW_PROXSENS);

    float sinxc = sin(angle_c*PI/180);
    float cosxc = cos(angle_c*PI/180);
    float sinxl = sin(angle_l*PI/180);
    float cosxl = cos(angle_l*PI/180);
    float sinxr = sin(angle_r*PI/180);
    float cosxr = cos(angle_r*PI/180);

    pta pivot;
    pivot.x = ImgPos.x+ImgPos.w/2;
    pivot.y = ImgPos.y+ImgPos.h/2-YOFFSET;
    for(int i = 0; i < sen_arr_size; i++)
    {
        setval(&(sen_arr[i]),pivot.x,pivot.y-i);
        rotate_point(&(sen_arr_c[i]),sen_arr[i],pivot,sinxc,cosxc);
        rotate_point(&(sen_arr_l[i]),sen_arr[i],pivot,sinxl,cosxl);
        rotate_point(&(sen_arr_r[i]),sen_arr[i],pivot,sinxr,cosxr);
    }
}

float smodi::check_visibility(float angle, float dist, surf_wrapper *bg)
{
    float sinx = sin(angle);
    float cosx = cos(angle);
    Uint32 valcomp = 0x00ffffff;
    pta pivot, tmp,rotated;
    pivot.x = ImgPos.x+ImgPos.w/2;
    pivot.y = ImgPos.y+ImgPos.h/2-YOFFSET;
    int q;
    for(q = (int)(COLLISION_RADIUS/scaleSize.x); q < dist_arr_size; q++)
    {
        tmp.x = pivot.x; tmp.y = pivot.y-q;
        rotate_point(&rotated,tmp,pivot,sinx,cosx);
        if(bg->get_px(rotated.x,rotated.y,WORKING)&valcomp)
            break;
    }
    if(((q*scaleSize.x+1.5*COLLISION_RADIUS)<dist)&&(dist>(1.5*COLLISION_RADIUS))) return -1.0*q*scaleSize.x;
    else return 1.0;
}

void smodi::init(int idx,pta scale,SDL_Rect bg)
{
    scaleSize.x = scale.x;
    scaleSize.y = scale.y;
    bgSize.w = bg.w;
    bgSize.h = bg.h;
    ImgPos.w = MODI_DIAMETER/scaleSize.x;
    ImgPos.h = MODI_DIAMETER/scaleSize.y;
    sen_arr_size = (int) (DEFAULT_RANGE/scaleSize.x);
    sen_arr = (pta *) malloc(sizeof(pta)*(sen_arr_size));
    sen_arr_l = (pta *) malloc(sizeof(pta)*(sen_arr_size));
    sen_arr_c = (pta *) malloc(sizeof(pta)*(sen_arr_size));
    sen_arr_r = (pta *) malloc(sizeof(pta)*(sen_arr_size));
    for(int i=0; i<sen_arr_size; i++)
        setval(&(sen_arr[i]),0.0,i);
    dist_arr_size = (int) (DETECTION_RANGE/scaleSize.x);
    // input data (from/to controller)
    data_to_vrep.left = DEFAULT_SPEED;
    data_to_vrep.right = (-1)*DEFAULT_SPEED;
    // virtual sensors
    data_to_control.senl = data_to_control.senc = data_to_control.senr = 0;
    data_to_control.pos.x = data_to_control.pos.y = data_to_control.pos.a = 0;
    for(int i = 0; i < NCSENS; i++) data_to_control.colsen[i] = 0;
    for(int i = 0; i < MAX_MODIES; i++)
    {
        data_to_control.m_pos[i].m = -1.0;
        data_to_control.m_pos[i].a = 0;
        data_to_control.o_pos[i].m = -1.0;
        data_to_control.o_pos[i].a = 0;
        data_to_control.mo_sur[i] = 0;
    }

    key_t shkey1 = SH_SENSORS+idx;
    key_t shkey2 = SH_MOTORS+idx;
    int shmid1, shmid2;
    if ((shmid1 = shmget(shkey1, sizeof(ModiSen), IPC_CREAT | 0666)) < 0)
    {
        printf(" error in modi %d pos %d, get sensor sh ",idx,shkey1);
        perror("shmget");
        exit(1);
    }
    if ((sh_data_to_control = (ModiSen*) shmat(shmid1, NULL, 0)) == (ModiSen *) -1)
    {
        printf(" error in modi %d",idx);
        perror("shmat");
        exit(1);
    }
    if ((shmid2 = shmget(shkey2, sizeof(motor_and_mo), IPC_CREAT | 0666)) < 0)
    {
        printf(" error in modi %d pos %d, get motor sh ",idx,shkey2);
        perror("shmget");
        exit(1);
    }
    if ((sh_data_to_vrep = (motor_and_mo*) shmat(shmid2, NULL, 0)) == (motor_and_mo *) -1)
    {
        printf(" error in modi %d",idx);
        perror("shmat");
        exit(1);
    }
    memset(sh_data_to_vrep,sizeof(motor_and_mo),0);
    pfile = NULL;
    data_to_control.id = idx;
    float da = 360.0/(NCSENS*3);
    for(int i = 0; i < (NCSENS*3); i++)
    {
        o_col_sens[i].a = valid_angle(i*da);
        col_detect[i] = 0;
    }
}

SDL_Rect smodi::getImgPos()
{
    return ImgPos;
}

void smodi::upd_data_from_vrep(float x, float y, float a)
{
    data_to_control.pos.x = x;
    data_to_control.pos.y = y;
    data_to_control.pos.a = a;

    ImgPos.x = (int) (bgSize.w/2 + x/scaleSize.x-0.5*MODI_DIAMETER/scaleSize.x);
    ImgPos.y = (int) (bgSize.h/2 - y/scaleSize.y-0.5*MODI_DIAMETER/scaleSize.y);
    ImgPos.w = MODI_DIAMETER/scaleSize.x;
    ImgPos.h = MODI_DIAMETER/scaleSize.y;
}

motor smodi::get_data_to_vrep()
{
    return data_to_vrep;
}

pta smodi::get_pos()
{
    return data_to_control.pos;
}

void smodi::upd_distances_to_modies(smodi *modi_arr, surf_wrapper *bg)
{
    float detection_rng = DETECTION_RANGE;
    for(int i=0; i<MAX_MODIES; i++)
    {
        if(!modi_arr[i].isEmpty()&&i!=data_to_control.id)
        {
            pta cpos = modi_arr[i].get_pos();
            pta mpos = data_to_control.pos;
            float dx = cpos.x-mpos.x;
            float dy = cpos.y-mpos.y;
            float mag = sqrt(dx*dx+dy*dy);
            if(mag>detection_rng)
            {
                data_to_control.m_pos[i].m = -1.0*detection_rng;
                continue;
            }
            float ang = valid_angle(180*atan2(dx,dy)/PI-mpos.a);
            float ang_from_axis = valid_angle(180*atan2(dx,dy)/PI);
            float check = check_visibility(PI*ang_from_axis/180,mag,bg);
            if(check < 0.0)
            {
                data_to_control.m_pos[i].m = check;
                continue;
            }
            else
            {
                data_to_control.m_pos[i].m = mag;
                data_to_control.m_pos[i].a = ang;
            }
        }
        else  data_to_control.m_pos[i].m = -1.0*detection_rng;
    }
}

void smodi::upd_distances_to_objects(mData *obj_arr, surf_wrapper *bg)
{
    float detection_rng = DETECTION_RANGE/2;
    for(int i=0; i<MAX_OBJECTS; i++)
    {
        if(obj_arr[i].id>=0)
        {
            pta mpos = data_to_control.pos;
            float dx = obj_arr[i].pos.x-mpos.x;
            float dy = obj_arr[i].pos.y-mpos.y;
            float mag = sqrt(dx*dx+dy*dy);
            if(mag>detection_rng)
            {
                data_to_control.o_pos[i].m = -1.0*detection_rng;
                continue;
            }
            float ang = valid_angle(180*atan2(dx,dy)/PI-mpos.a);
            float ang_from_axis = valid_angle(180*atan2(dx,dy)/PI);
            float check = check_visibility(PI*ang_from_axis/180,mag,bg);
            //if(i==0) printf("%f\n",check);//data_to_control.o_pos[0].m );
            if(check < 0.0)
            {
                data_to_control.o_pos[i].m = check;
                continue;
            }
            else
            {
                data_to_control.o_pos[i].m = mag;
                data_to_control.o_pos[i].a = ang;
            }
        }
        else  data_to_control.o_pos[i].m = -1.0*detection_rng;
    }

}

void smodi::draw_rays(surf_wrapper *bg)
{
    for(int i=0; i<MAX_MODIES; i++)
    {
        if(i!=data_to_control.id&&(data_to_control.m_pos[i].m>=0.0))
        {
            int arr_pos = (int) (data_to_control.m_pos[i].m/scaleSize.x);
            arr_pos = (arr_pos>=0)? arr_pos: -1*arr_pos;
            float angle = valid_angle(data_to_control.m_pos[i].a+data_to_control.pos.a);
            float sinx = sin(angle*PI/180);
            float cosx = cos(angle*PI/180);
            Uint32 color = 0x0000ffff;
            pta pivot, tmp, rotated;
            pivot.x = ImgPos.x+ImgPos.w/2;
            pivot.y = ImgPos.y+ImgPos.h/2-YOFFSET;
            for(int q = 0; q < arr_pos; q++)
            {
                tmp.x = pivot.x; tmp.y = pivot.y-q;
                rotate_point((&rotated),tmp,pivot,sinx,cosx);
                bg->put_px(rotated.x,rotated.y,color);
            }

        }
        if(data_to_control.m_pos[i].m<0)
        {
            data_to_control.m_pos[i].m = -1.0;
            data_to_control.m_pos[i].a = 0;
        }
    }
    for(int i=0; i<MAX_OBJECTS; i++)
    {
        if(data_to_control.o_pos[i].m>=0.0)
        {
            int arr_pos = (int) (data_to_control.o_pos[i].m/scaleSize.x);
            arr_pos = (arr_pos>=0)? arr_pos: -1*arr_pos;
            float angle = valid_angle(data_to_control.o_pos[i].a+data_to_control.pos.a);
            float sinx = sin(angle*PI/180);
            float cosx = cos(angle*PI/180);
            Uint32 color = 0x00ffff00;
            pta pivot, tmp, rotated;
            pivot.x = ImgPos.x+ImgPos.w/2;
            pivot.y = ImgPos.y+ImgPos.h/2-YOFFSET;
            for(int q = 0; q < arr_pos; q++)
            {
                tmp.x = pivot.x; tmp.y = pivot.y-q;
                rotate_point((&rotated),tmp,pivot,sinx,cosx);
                bg->put_px(rotated.x,rotated.y,color);
            }

        }
        if(data_to_control.o_pos[i].m<0.0)
        {
            data_to_control.o_pos[i].m = -1.0;
            data_to_control.o_pos[i].a = 0;
        }
    }
}

void smodi::upd_motor_from_shm()
{
    if(sh_data_to_vrep!=NULL)
    {
        memcpy(&data_to_vrep,&(sh_data_to_vrep->mot),sizeof(motor));
        memcpy(&(data_to_control.mo),&(sh_data_to_vrep->mo),sizeof(float));
    }
}

void smodi::upd_sensors_shm()
{
    memcpy(sh_data_to_control,&data_to_control,sizeof(ModiSen));
}

void smodi::setMotors(float left, float right)
{
    data_to_vrep.left = left;
    data_to_vrep.right = right;
}

void smodi::make_available()
{
    data_to_control.id = -1;
}

bool smodi::isEmpty()
{
    return (data_to_control.id==(-1));
}

void smodi::printUtil(int what)
{
    if(what&MODI_DISTANCES)
    {
        for(int j = 0; j < MAX_MODIES; j++)
        {
           printf("(%.3f,%.3f) ",data_to_control.m_pos[j].m,data_to_control.m_pos[j].a);
        }
        printf("\n");
        fflush(stdin);
    }
    if(what&OBJ_DISTANCES)
    {
        for(int j = 0; j < MAX_MODIES; j++)
        {
           printf("(%.3f,%.3f) ",data_to_control.o_pos[j].m,data_to_control.o_pos[j].a);
        }
        printf("\n");
        fflush(stdin);
    }
    if(what&COL_SENSORS)
    {
        for(int j = 0; j < NCSENS; j++)
        {
           printf("%d ",data_to_control.colsen[j]);
        }
        printf("\n");
        fflush(stdin);
    }
}

void smodi::upd_prox_sen(surf_wrapper* bg)
{
    int min_to_sens = MIN_SENS_DIST/scaleSize.y;
    upd_rotpx();
    Uint32 valcomp = 0x00ffffff; // mask with white
    float dx,dy;
    for(i = min_to_sens; i < sen_arr_size; i++)
         if(bg->get_px(sen_arr_c[i],WORKING)&valcomp) break;
    if(i<sen_arr_size)
    {
        ii = i;
        dx = (sen_arr_c[i].x-sen_arr_c[0].x)*scaleSize.x;
        dy = (sen_arr_c[i].y-sen_arr_c[0].y)*scaleSize.y;
        data_to_control.senc = sqrt(dx*dx+dy*dy);
    }
    else
    {
        data_to_control.senc = DEFAULT_RANGE;
        ii = sen_arr_size-1;
    }
    for(j = min_to_sens; j < sen_arr_size; j++)
         if(bg->get_px(sen_arr_l[j],WORKING)&valcomp) break;
    if(j<sen_arr_size)
    {
        jj = j;
        dx = (sen_arr_l[j].x-sen_arr_l[0].x)*scaleSize.x;
        dy = (sen_arr_l[j].y-sen_arr_l[0].y)*scaleSize.y;
        data_to_control.senl = sqrt(dx*dx+dy*dy);
    }
    else
    {
        data_to_control.senl = DEFAULT_RANGE;
        jj = sen_arr_size-1;
    }
    for(k = min_to_sens; k < sen_arr_size; k++)
         if(bg->get_px(sen_arr_r[k],WORKING)&valcomp) break;
    if(k<sen_arr_size)
    {
        kk = k;
        dx = (sen_arr_r[k].x-sen_arr_r[0].x)*scaleSize.x;
        dy = (sen_arr_r[k].y-sen_arr_r[0].y)*scaleSize.y;
        data_to_control.senr = sqrt(dx*dx+dy*dy);
    }
    else
    {
        data_to_control.senr = DEFAULT_RANGE;
        kk = sen_arr_size-1;
    }
}

void smodi::draw_prox_sen(surf_wrapper *bg)
{
    int min_to_sens = MIN_SENS_DIST/scaleSize.y;
    int q = ii;
    for(int i=min_to_sens; i<q; i++)
        bg->put_px(sen_arr_c[i].x,sen_arr_c[i].y,COLOR_RED);
    for(int j=min_to_sens; j<jj; j++)
        bg->put_px(sen_arr_l[j].x,sen_arr_l[j].y,COLOR_CYAN);
    for(int k=min_to_sens; k<kk; k++)
        bg->put_px(sen_arr_r[k].x,sen_arr_r[k].y,COLOR_CYAN);
}

void smodi::upd_col_sen(surf_wrapper *bg)
{
    Uint32 valcomp = 0x00ffffff; // mask with white
    pta tmp;
    pta pivot;
    float current_angle = data_to_control.pos.a;
    pivot.x = ImgPos.x+(int)(ImgPos.w*0.5);
    pivot.y = ImgPos.y+(int)(ImgPos.h*0.5);
    for(int i = 0; i < NCSENS*3; i++) col_detect[i] = 0;
    for(int i = 0; i < NCSENS*3; i++)
    {
        tmp.x = pivot.x;
        tmp.y = pivot.y - COLLISION_RADIUS/scaleSize.y;
        float sinx = sin(valid_angle(current_angle+o_col_sens[i].a)*PI/180);
        float cosx = cos(valid_angle(current_angle+o_col_sens[i].a)*PI/180);
        rotate_point(&col_sens[i],tmp,pivot,sinx,cosx);
        if(bg->get_px(col_sens[i],WORKING)&valcomp) col_detect[i] = 1;
    }
    if(col_detect[3*NCSENS-1]||col_detect[0]||col_detect[1]) data_to_control.colsen[0] = 1;
    else data_to_control.colsen[0] = 0;
    for(int i=1; i<NCSENS;i++)
    {
        if(col_detect[3*i-1]||col_detect[3*i]||col_detect[3*i+1]) data_to_control.colsen[i] = 1;
        else data_to_control.colsen[i] = 0;
    }
}

void smodi::draw_col_sen(surf_wrapper *bg)
{
    for(int i=1; i<NCSENS*3;i++)
        bg->put_3x3px(col_sens[i].x,col_sens[i].y,COLOR_MAGENTA);
}

float smodi::getMO()
{
    return data_to_control.mo;
}

void smodi::upd_mo_array(smodi *modi_arr)
{
    for(int i=0; i< MAX_MODIES; i++)
    {
        if(data_to_control.m_pos[i].m>=0)
        {
            data_to_control.mo_sur[i] = modi_arr[i].getMO();
        }
    }
}


/*
// initialize the logger for this modi WARNING: errors are not captured
void modi::initLogger(char *filename)
{
    pfile = fopen(filename,"w");
}
// reports the current state of the robot
void modi::report(bool copy2file,bool verbose)
{
    if(copy2file)
        if(pfile!=NULL) fprintf(pfile, "%f,%f,%f\n",data.pos.x, data.pos.y, data.pos.a);
    if(verbose)
        printf("modi %02d: x:%f, y:%f, a:%f\n",sens.id,data.pos.x, data.pos.y, data.pos.a);

}
void modi::setshdata(int inkeydata,int inkeycmd)
{
    keydata = (key_t) inkeydata;
    if ((shmid = shmget(keydata, sizeof(mData), IPC_CREAT | 0666)) < 0)
    {
        perror("shmget");
        exit(1);
    }
    if ((sdata = (mData*) shmat(shmid, NULL, 0)) == (mData *) -1)
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
    printf("shared memories with keys %d and %d created.\n",keydata,keycmd);
}
void modi::updateshdata(bool comrdy)
{
    if(comrdy)
        data.flag = (data.flag&FRM_FLAG)^FRM_FLAG+(data.flag&COM_FLAG)^COM_FLAG;
    else
        data.flag = (data.flag&FRM_FLAG)^FRM_FLAG;
    memcpy(sdata,&data,sizeof(mData));
}
*/
