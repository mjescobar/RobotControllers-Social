/* define these constants to activate the respective functions */
// #define DO_COMM          // if you want to perform the xbee communication with the robot
// #define DO_REPORT        // if you want to write in a file the position of the robot
// #define USE_SENSORS      // if you want to use, update and draw the sensors.
                            // it also change the declaration of mDataf.

#ifndef _BMAP_H_
#define _BMAP_H_
#include <string>


typedef struct pt
{
    int x;
    int y;
} pt;

typedef struct{
    float x;
    float y;
    float a;
} pta;


class bmap
{
public:
    bmap();
    bmap(const char* filename,int inoffx, int inoffy, int framewidth, int frameheight);
    void load(const char* filename);
    void setval(int x, int y, unsigned char * value);
    unsigned char getOriginal(int x, int y,int ch);
    unsigned char getWorking(int x, int y,int ch);
    void overlayInto( unsigned char * dataPtr, int dst_image_width,bool tmp);
    void overlaySqFloating(bmap other, int x, int y);
    int getFramesize();
    int getImgAsize();
    int getWidth();
    int getHeight();
    int getOffx();
    int getOffy();
    unsigned char * getOriginalPtr();
    unsigned char * getWorkingPtr();
    void clearWorking();

private:
    int width;
    int height;
    int a_size;
    int framesize;

    unsigned char * image;
    unsigned char * tmp;
    int channels;
    int offx;
    int offy;

};
#endif
