#ifndef BEHAVIORSH_H
#define BEHAVIORSH_H
#include "defs.h"
#include <random>
// behavioral schemata

#define DEBUG 0

class behaviorsh
{
    public:
        behaviorsh();
        virtual ~behaviorsh();
        void update(int *e, float MO, int hold_behavior, int reasonedb, int modi2follow, int obj2follow);
        int GetBehavior() { return sel_behavior; }
        float GetBF();
    protected:
    private:
        int behaviors[MAX_BEHAVIORS];       // which behavior will be developed
        float behaviorsMO[MAX_BEHAVIORS];   // motivational orientation of each behavior
        float behaviorsPR[MAX_BEHAVIORS];   // behaviors priority
        float behaviorsPR_def[MAX_BEHAVIORS];   // behaviors priority by default
        int sel_behavior;
        void load_orientations();
        void load_priorities();
        void act(int where,float factor, int val = 1);
        void mo_bias(float MO);
        void normalize();
        int resolveCollisions();
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution;
};

#endif // BEHAVIORSH_H
