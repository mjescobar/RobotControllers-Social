#ifndef OBJECT_H
#define OBJECT_H
#include "modi.h"

class object
{
    public:
        object();
        object(int pattid);
        virtual ~object();
        int getSocket();
        int getPattid();
        void setpos(int inx, int iny,double incf);
        pta getpos();
        void setSocket(int i,object *ref);
        bool isVisible();
        int getKidx();
        void setKidx(int value);
        void updateAngle(double angval);
    protected:
    private:
        mDataf data;
        // fiducial detection
        int patt_id;
        int k_index;
        double cf;
        // socket info
        int sockid;
        pthread_t tid;
        int thrd;
};

#endif // OBJECT_H
