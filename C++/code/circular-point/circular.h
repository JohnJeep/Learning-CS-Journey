/*
 * @Author: your name
 * @Date: 2020-06-01 14:09:24
 * @LastEditTime: 2020-06-01 15:48:38
 * @LastEditors: Please set LastEditors
 * @Description: 圆类的声明
 */ 
#ifndef __CIRCULAR_H
#define __CIRCULAR_H
#include "point.h"
#include <iostream>
using namespace std;
class Circular
{
private:
    int x0;
    int y0;
    int r0;

public:
    void setCircle(int c_r0, int c_x0, int c_y0);
    void getCircle();
    int judgePosition(Point& p);

    
    Circular(/* args */);
    ~Circular();
};

Circular::Circular(/* args */)
{
}

Circular::~Circular()
{
}

#endif // !__CIRCULAR_H




