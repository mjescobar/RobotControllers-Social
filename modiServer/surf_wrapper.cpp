#include "surf_wrapper.h"

surf_wrapper::surf_wrapper()
{
}
surf_wrapper::~surf_wrapper()
{
    SDL_FreeSurface(original);
    SDL_FreeSurface(working);
    original = NULL;
    working = NULL;
}
void surf_wrapper::init(char *filename, SDL_Surface *window_surface, bool haveAlpha)
{
    SDL_Surface* loadedSurface = SDL_LoadBMP( filename );
    if( loadedSurface == NULL )
    {
        printf( "Unable to load image %s! SDL Error: %s\n", filename, SDL_GetError() );
    }
    else
    {
        if(haveAlpha)
        {
            /* Set transparent pixel as the pixel at (0,0) */
            if (loadedSurface->format->palette)
            {
                SDL_SetColorKey(loadedSurface, 1, *(Uint8 *) loadedSurface->pixels);
            }
            else
            {
                switch (loadedSurface->format->BitsPerPixel)
                {
                case 15:
                    SDL_SetColorKey(loadedSurface, 1, (*(Uint16 *) loadedSurface->pixels) & 0x00007FFF);
                    break;
                case 16:
                    SDL_SetColorKey(loadedSurface, 1, *(Uint16 *) loadedSurface->pixels);
                    break;
                case 24:
                    SDL_SetColorKey(loadedSurface, 1, (*(Uint32 *) loadedSurface->pixels) & 0x00FFFFFF);
                    break;
                case 32:
                    SDL_SetColorKey(loadedSurface, 1, *(Uint32 *) loadedSurface->pixels);
                    break;
                }
            }
        }
        //Convert surface to screen format
        original = SDL_ConvertSurface( loadedSurface, window_surface->format,0 );
        working = SDL_ConvertSurface( loadedSurface, window_surface->format,0 );
        if( (original == NULL)||(working == NULL) )
        {
            printf( "Unable to optimize image %s! SDL Error: %s\n", filename, SDL_GetError() );
        }
        //Get rid of old loaded surface
        SDL_FreeSurface( loadedSurface );
        dim.w = original->w;
        dim.h = original->h;
    }
 }

SDL_Rect surf_wrapper::get_size()
{
    return dim;
}

void surf_wrapper::put_px(int x, int y, Uint32 val)
{
    if((x>=0)&&(x<working->w)&&(y>=0)&&(y<working->h))
        ((Uint32 *)working->pixels)[ ( y * working->w ) + x ] = val;
}
void surf_wrapper::put_3x3px(int x, int y, Uint32 val)
{
    if((x>0)&&(x<(working->w-1))&&(y>0)&&(y<(working->h-1)))
    {
        ((Uint32 *)working->pixels)[ ( y * working->w ) + x ] = val;
        ((Uint32 *)working->pixels)[ ( (y-1) * working->w ) + x ] = val;
        ((Uint32 *)working->pixels)[ ( (y+1) * working->w ) + x ] = val;
        ((Uint32 *)working->pixels)[ ( y * working->w ) + (x-1) ] = val;
        ((Uint32 *)working->pixels)[ ( y * working->w ) + (x+1) ] = val;
    }
}

Uint32 surf_wrapper::get_px(int x, int y, int which)
{
    Uint32 val;
    if((x>=0)&&(x<original->w)&&(y>=0)&&(y<original->h))
    {
        if(which==ORIGINAL)
            val = ((Uint32 *)original->pixels)[ ( y * original->w ) + x ];
        if(which==WORKING)
            val = ((Uint32 *)working->pixels)[ ( y * working->w ) + x ];
        return val;
    }
    else return 0x0;
}
Uint32 surf_wrapper::get_px(pta pos, int which)
{
    int x = pos.x;
    int y = pos.y;
    Uint32 val = 0x0;
    if((x>=0)&&(x<original->w)&&(y>=0)&&(y<original->h))
    {
        if(which==ORIGINAL)
            val = ((Uint32 *)original->pixels)[y*(original->w)+x];
        if(which==WORKING)
            val = ((Uint32 *)working->pixels)[y*(working->w)+x];
        return val;
    }
    else return val;
}

void surf_wrapper::rot(float angle)
{
    SDL_FillRect( working, NULL, *(Uint32 *) working->pixels);
    float sinx = sin(angle);
    float cosx = cos(angle);
    float x0 = dim.w/2.0;
    float y0 = dim.h/2.0;
    int x1,x2,y1,y2;
    for(x1 = 0; x1 < dim.w; x1++)
    {
        for(y1 = 0; y1 < dim.h; y1++)
        {
            x2 = (int)(cosx*(x1-x0)-sinx*(y1-y0)+x0);
            y2 = (int)(sinx*(x1-x0)+cosx*(y1-y0)+y0);
            if((x2>0)&&(x2<(dim.w-1))&&(y2>0)&&(y2<(dim.h)))
                put_px(x2,y2,get_px(x1,y1,ORIGINAL));
        }
    }
    for(x1 = 1; x1 < dim.w*dim.h-1; x1++)
        if(((Uint32*)working->pixels)[x1]==((Uint32 *)working->pixels)[0])
        {
            if(((Uint32*)working->pixels)[x1-1]==((Uint32 *)working->pixels)[0])
                ((Uint32*)working->pixels)[x1] = ((Uint32*)working->pixels)[x1-1];
            else
                ((Uint32*)working->pixels)[x1] = ((Uint32*)working->pixels)[x1+1];
        }
}

SDL_Surface* surf_wrapper::get(int which)
{
    if(which==WORKING) return working;
    if(which==ORIGINAL) return original;
    else return NULL;
}

void surf_wrapper::reset()
{
    SDL_BlitSurface(original,NULL,working,NULL);
}
