#ifndef VPOINT_H
#define VPOINT_H

#include<iostream>
using namespace std;




class VPoint
{
public:
    float x;
    float y;
    float z;
	
};

class VPointRGB : public VPoint
{
public:
    unsigned char R;
    unsigned char G;
    unsigned char B;

};

class VPointI : public VPoint
{
public:
    float intensity;  //0-255
	
};

class VPointIL : public VPointI
{
public:
    int label;  //

};



#endif // VPOINT_H
