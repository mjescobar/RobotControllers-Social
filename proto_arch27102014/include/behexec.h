#ifndef BEHEXEC_H
#define BEHEXEC_H
#include "defs.h"
/*
#define FULL_STOP       x  ok    #define WALL_FOLLOW     x ok
#define REACH_OBJ       x  ok    #define EVADE_OBJ       x ok
#define FREE_EXP        x  ok    #define MARK_CORNERS    o no
#define TURN_180        x  ok    #define MODI_FOLLOW     x ok
#define MORE_SPD        x  ok    #define LESS_SPD        x ok
#define EVADE_OBST      x  ok
*/
typedef struct{
    int prev_spd;
} struct_full_stop;

typedef struct{
    float minNoObj;     // cm -> min. distance to avoid collision on the front
    float minObjLat;    //  min. distance to avoid collision on the side
    float critObjLat;   //  critical minimum distance to an object
    float hist_norm;    // hysteresis normal
    float hist_crit;    // hysteresis critical
    int cState;
} struct_free_exp;

typedef struct{
    float minNoObj;     // cm -> min. distance to avoid collision on the front
    float minObjLat;    // min. distance to avoid collision on the side
    float critObjLat;   // critical minimum distance to an object
    float hist_norm;    // hysteresis normal
    float hist_crit;    // hysteresis critical
    int cState;
    int fcd;
    float start_angle;
    float selected_angle;
    float first_dist;
} struct_wall_follow_old;

typedef struct{
    int direction;
    float close_dist;
    float crit_dist;
    float prev_dist;
    int recover_wall;
} struct_wall_follow;

typedef struct{
    float  delta;
} struct_more_spd;

typedef struct{
    float  delta;
} struct_less_spd;

typedef struct{
    float  current_speed;
    float start_angle;
    float prev_angle;
    float target_angle;
    int cState;
    int sturn;
} struct_turn_180;

typedef struct {
    int obj2detect;
    int cState;
    float original_spd;
} struct_reach_obj;

typedef struct {
    int obj2detect;
    int cState;
    float original_spd;
} struct_evade_obj;

typedef struct {
    int modi2detect;
    int cState;
    float original_spd;
    float prev_dist;
} struct_modi_follow;

typedef struct {
    int cState;
} struct_evade_obst;

class behexec
{
    public:
        behexec();
        void initShm(int idx);
        virtual ~behexec();
        int GetLM();
        int GetRM();
        void update(int sel_behavior);
        void upd_motor_and_mo_to_shm(float mo);
        void setSens(ModiSen *addr) { rob = addr; }
        int GetHoldBehavior() { return current_behavior; }
        void reset(int modi2detect, int obj2detect);
    protected:
    private:
        void f_full_stop();
        void f_free_exp();
        void f_more_spd();
        void f_less_spd();
        void f_turn_180();
        void f_wall_follow();
        void f_reach_obj();
        void f_evade_obj();
        void f_modi_follow();
        void f_evade_obst();

        int current_behavior;
        int prev_behavior;
        motor mot;
        motor_and_mo *motmosh;
        float default_speed;
        int hold_behavior;
        void setVel(float ml, float mr);
        ModiSen *rob;
        struct_full_stop s_full_stop;
        struct_free_exp s_free_exp;
        struct_more_spd s_more_spd;
        struct_less_spd s_less_spd;
        struct_turn_180 s_turn_180;
        struct_wall_follow s_wall_follow;
        struct_reach_obj s_reach_obj;
        struct_evade_obj s_evade_obj;
        struct_modi_follow s_modi_follow;
        struct_evade_obst s_evade_obst;
};

#endif // BEHEXEC_H
