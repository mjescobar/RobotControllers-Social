#ifndef PERCEPTION_H
#define PERCEPTION_H
#include <iostream>
#include <vector>
#include "defs.h"


class perception
{
    public:
        // constructor
        perception();
        perception(ModiSen* robot_inputs);
        virtual ~perception();

        // multi-purpose functions
        void initShm(int idx);
        void read_sensor_from_shm();
        void update(float MO);
        void reset(int low, int high);
        void setSens(ModiSen *addr) { inref = addr; }
        float GetEO() { return EO; }
        int* getEvents();

    protected:
    private:
        ModiSen* inref;
        ModiSen* shref;
        int events_act[MAX_EVENTS];
        float mo_bias;
        float EO;
};

#endif // PERCEPTION_H
