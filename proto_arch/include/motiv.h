#ifndef MOTIV_H
#define MOTIV_H
#include "defs.h"


class motiv
{
    public:
        motiv();
        virtual ~motiv();

        void update();
        void update(ModiSen* rob,float inBF, float inEO, float inRO);
        void reset(float initval,float *inweights);
        void SetWeight(int which, float value) { weights[which] = value;}
        float GetRO() { return factors[1]; }
        void SetRO(float val) { factors[1] = val; }
        float GetBF() { return factors[2]; }
        void SetBF(float val) { factors[2] = val; }
        float GetMO(bool updated = false);
        void SetMO(float val) { factors[0] = val; }
        float GetEO() { return factors[3]; }
        void SetEO(float val) { factors[3] = val; }
    protected:
    private:
        float w_norm;
        float weights[4]; // w1, w2, w3, w4
        float factors[4]; // MO, RO, BF, EO
};


#endif // MOTIV_H
