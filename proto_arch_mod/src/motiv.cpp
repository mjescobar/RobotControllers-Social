#include "../include/motiv.h"
#include <stdio.h>

motiv::motiv()
{
    //ctor
    reset(0.0,NULL);
}

/*  the commented lines were commented on 11-09-2014.
    now the motivational orientation has a linear increasing/decreasing.
*/

void motiv::update()
{
    factors[M_MO] = weights[M_WRO]*factors[M_RO] + weights[M_WBF]*factors[M_BF] + weights[M_WMO]*factors[M_MO] + weights[M_WEO]*factors[M_EO];
    //factors[M_MO] = (factors[M_MO]==0.0)?factors[M_MO] : factors[M_MO]/absv(factors[M_MO]);
    factors[M_MO] = factors[M_MO]/w_norm;
}
void motiv::update(ModiSen* rob,float inBF, float inEO, float inRO)
{
    factors[M_BF] = inBF;
    factors[M_EO] = inEO;
    factors[M_RO] = inRO;

    float prev = factors[M_MO];

    // lines added at 21-12-2014
    factors[M_EO] = ((factors[M_EO]*prev > 0.0)||(factors[M_EO] == 0.0))? prev : factors[M_EO];
    factors[M_BF] = (factors[M_BF] == 0.0)? prev : factors[M_BF];

    float f1 = weights[M_WRO]*factors[M_RO];
    float f2 = weights[M_WBF]*factors[M_BF];
    float f3 = weights[M_WEO]*factors[M_EO];
    float upd = (f1+f2+f3)/(weights[M_WRO]+weights[M_WBF]+weights[M_WEO]+EPSIL);
    float rate = weights[M_WMO];

    factors[M_MO] = (1-rate)*upd+rate*prev;
    if(factors[M_MO]>1.0) factors[M_MO] = 1.0;
    if(factors[M_MO]<-1.0) factors[M_MO] = -1.0;
   // if(rob->id==0)
    //    printf("%d:\t%f\t%f\t%f\t%f\n",rob->id,weights[M_WBF]*factors[M_BF],weights[M_WEO]*factors[M_EO],upd, factors[M_MO]);

    rob->mo = (factors[M_MO]==0.0)?factors[M_MO] : factors[M_MO]/absv(factors[M_MO]);



    /*  THIS IS THE OLD VERSION OF THIS FUNCTION

    factors[M_BF] = inBF;
    factors[M_EO] = inEO;
    factors[M_RO] = inRO;

    factors[M_MO] = weights[M_W1]*factors[M_RO] + weights[M_W2]*factors[M_BF] + weights[M_W3]*factors[M_MO] + weights[M_W4]*factors[M_EO];
    //factors[M_MO] = (factors[M_MO]==0.0)?factors[M_MO] : factors[M_MO]/absv(factors[M_MO]);
    //rob->mo = factors[M_MO];
    factors[M_MO] = factors[M_MO]/w_norm;
    printf(" mo:%f\n",factors[M_MO]);
    rob->mo = (factors[M_MO]==0.0)?factors[M_MO] : factors[M_MO]/absv(factors[M_MO]);

    END OF COMMENT */
}

void motiv::reset(float initval,float *inweights)
{
    for(int i = 0; i < 4; i++)
    {
        factors[i] = 0;
    }
    factors[M_MO] = initval;
    if(inweights==NULL)
    {
        weights[M_WRO] = 0.0; /// M_RO
        weights[M_WBF] = 0.3 ; /// M_BF
        weights[M_WMO] = 0.992; /// M_MO
        weights[M_WEO] = 0.7; /// M_EO
    }
    else
    {
        weights[M_WRO] = 0.0; /// M_RO
        weights[M_WMO] = inweights[0]; /// M_MO
        weights[M_WBF] = inweights[1]; /// M_BF
        weights[M_WEO] = inweights[2]; /// M_EO
    }
    w_norm = 0;
    for(int i=0; i < 4; i++)
        w_norm = w_norm + weights[i];
}

float motiv::GetMO(bool updated)
{
    if(updated) update();
    return factors[M_MO];
}

motiv::~motiv()
{
    //dtor
}
