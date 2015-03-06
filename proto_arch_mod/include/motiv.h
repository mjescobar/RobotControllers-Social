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
        float GetRO() { return factors[M_RO]; }
        void SetRO(float val) { factors[M_RO] = val; }
        float GetBF() { return factors[M_BF]; }
        void SetBF(float val) { factors[M_BF] = val; }
        float GetMO(bool updated = false);
        void SetMO(float val) { factors[M_MO] = val; }
        float GetEO() { return factors[M_EO]; }
        void SetEO(float val) { factors[M_EO] = val; }
    protected:
    private:
        float w_norm;
        float weights[4]; // w1, w2, w3, w4
        float factors[4]; // MO, RO, BF, EO
};


#endif // MOTIV_H
