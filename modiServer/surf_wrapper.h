
#ifndef SURF_WRAPPER_H
#define SURF_WRAPPER_H

#include "SDL.h"
#include <cmath>

#define ORIGINAL 1
#define WORKING 0

#define COLOR_RED       0x00ff0000
#define COLOR_GREEN     0x0000ff00
#define COLOR_BLUE      0x000000ff
#define COLOR_CYAN      0x0000ffff
#define COLOR_MAGENTA   0x00ff00ff
#define COLOR_YELLOW    0x00ffff00

// 2D point + angle
typedef struct{
    float x;
    float y;
    float a;
} pta;

typedef struct{
    float m;
    float a;
} vec;

class surf_wrapper
{
    public:
        surf_wrapper();
        void init(char *filename, SDL_Surface *window_surface, bool haveAlpha=false);
        SDL_Rect get_size();
        void put_px(int x, int y, Uint32 val);
        void put_3x3px(int x, int y, Uint32 val);
        Uint32 get_px(int x, int y,int which);
        Uint32 get_px(pta pos,int which);
        void rot(float angle);
        SDL_Surface* get(int which = WORKING);
        void reset();
        virtual ~surf_wrapper();
    protected:
    private:
        SDL_Surface* original;
        SDL_Surface* working;
        SDL_Rect dim;
};

#endif // SURF_WRAPPER_H
