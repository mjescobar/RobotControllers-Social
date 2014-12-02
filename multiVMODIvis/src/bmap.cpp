#include "bmap.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

bmap::bmap()
{
    width = 0;
    height = 0;
    a_size = 0;
    framesize = 0;
    image = NULL;
    channels = 3;
    offx = 0;
    offy = 0;
}
bmap::bmap(const char* filename,int inoffx, int inoffy, int framewidth, int frameheight)
{
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header
    // extract image height and width from header
    width = *(int*)&info[18];
    height = *(int*)&info[22];
    printf("bitmap %s size: %d,%d\n",filename,width,height);
    // src array size
    a_size = 3 * width * height;
    //dst array size
    framesize = 3 * framewidth * frameheight;
    // allocate 3 bytes per pixel
    image = (unsigned char*) malloc(sizeof(unsigned char)*a_size);
    tmp = (unsigned char*) malloc(sizeof(unsigned char)*a_size);
    fread(image, sizeof(unsigned char), a_size, f); // read the rest of the data at once
    fclose(f);
    memcpy(tmp, image,a_size);
    offx = inoffx;
    offy = inoffy;
}

void bmap::load(const char* filename)
{
    FILE* f = fopen(filename, "rb");
    unsigned char info[54];
    fread(info, sizeof(unsigned char), 54, f); // read the 54-byte header
    // extract image height and width from header
    width = *(int*)&info[18];
    height = *(int*)&info[22];
    // src array size
    a_size = 3 * width * height;
    // allocate 3 bytes per pixel
    image = (unsigned char*) malloc(sizeof(unsigned char)*a_size);
    tmp = (unsigned char*) malloc(sizeof(unsigned char)*a_size);
    fread(image, sizeof(unsigned char), a_size, f); // read the rest of the data at once
    fclose(f);
    memcpy(tmp, image,a_size);

}

void bmap::setval(int x, int y, unsigned char *value)
{
    if(x>0&&y>0&&x<width&&y<height)
    {
        image[width*channels*y+x*channels] = value[0];
        image[width*channels*y+x*channels+1] = value[1];
        image[width*channels*y+x*channels+2] = value[2];
    }
}

unsigned char bmap::getOriginal(int x, int y, int ch)
{
    return image[width*3*(y-offy) + 3*(x-offx)+ch];
}

unsigned char bmap::getWorking(int x, int y, int ch)
{
    return tmp[width*3*(y-offy) + 3*(x-offx)+ch];
}

void bmap::overlayInto( unsigned char * dataPtr, int dst_image_width,bool iftmp)
{
    int i;
    if(iftmp)
    {
        #pragma omp parallel for schedule (dynamic, 50)
        for(i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                if(tmp[width*3*i+3*j+2]||tmp[width*3*i+3*j+1]||tmp[width*3*i+3*j])
                {
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3+2] =  tmp[width*3*i+3*j+2];
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3+1] = tmp[width*3*i+3*j+1];
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3] = tmp[width*3*i+3*j];
                }
            }
        }
    }
    else
    {
        #pragma omp parallel for schedule (dynamic, 50)
        for(i = 0; i < height; i++)
        {
            for(int j = 0; j < width; j++)
            {
                if(image[width*3*i+3*j+2]||image[width*3*i+3*j+1]||image[width*3*i+3*j])
                {
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3+2] =  image[width*3*i+3*j+2];
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3+1] = image[width*3*i+3*j+1];
                    dataPtr[dst_image_width*3*(offy+i)+(offx+j)*3] = image[width*3*i+3*j];
                }
            }
        }
    }
}


void bmap::overlaySqFloating(bmap other, int inx, int iny)
{
    pt pos;
    pos.x = inx;
    pos.y = iny;
    int ofx = pos.x;
    int ofy = pos.y;
    int gap = width/2;
    unsigned char* wimg = other.getWorkingPtr();
    int ooffx = other.getOffx();
    int ooffy = other.getOffy();
    int owidth = other.getWidth();
    int oheight = other.getHeight();
    int i,j;
    for(i = 0; i < height; i++)
    {
        for(j = 0; j < width; j++)
        {
            if((ofx-ooffx)>gap&&(ofy-ooffy)>gap&&(ofx-ooffx)<(owidth-gap)&&(ofy-ooffy)<(oheight-gap))
            {
                if(image[width*3*i+3*j+2]) wimg[owidth*3*(ofy-gap-ooffy+i)+(ofx-gap-ooffx+j)*3+2] =  image[width*3*i+3*j+2];
                if(image[width*3*i+3*j+1]) wimg[owidth*3*(ofy-gap-ooffy+i)+(ofx-gap-ooffx+j)*3+1] = image[width*3*i+3*j+1];
                if(image[width*3*i+3*j]) wimg[owidth*3*(ofy-gap-ooffy+i)+(ofx-gap-ooffx+j)*3] = image[width*3*i+3*j];
            }
        }
    }

}

int bmap::getImgAsize()
{
    return a_size;
}

int bmap::getFramesize()
{
    return framesize;
}
unsigned char * bmap::getOriginalPtr()
{
    return image;
}
unsigned char * bmap::getWorkingPtr()
{
    return tmp;
}
void bmap::clearWorking()
{
    memcpy(tmp,image,a_size);
}
int bmap::getWidth()
{
    return width;
}
int bmap::getHeight()
{
    return height;
}
int bmap::getOffx()
{
    return offx;
}
int bmap::getOffy()
{
    return offy;
}
