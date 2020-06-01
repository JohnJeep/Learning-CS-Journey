/*
 * @Author: JohnJeep
 * @Date: 2020-06-01 14:08:57
 * @LastEditTime: 2020-06-01 20:04:06
 * @LastEditors: Please set LastEditors
 * @Description: 圆类的实现
 */ 
#include "circular.h"
#include "point.h"
using namespace std;
#include <iostream>

void Circular::setCircle(int c_r0, int c_x0, int c_y0)
{
    x0 = c_x0;
    y0 = c_y0;
    r0 = c_r0;
}

void Circular::getCircle()
{
}

int Circular::judgePosition(Point& p)
{
    int distance = 0;
    distance = (p.getPointX1() - x0)*(p.getPointX1() - x0) + 
               (p.getPointY1()-y0)*(p.getPointY1()-y0);
    if (distance < r0*r0)
    {
        return 1; // 圆内
    }
    else
    {
        return 0; // 圆外
    }
}

Circular::Circular(/* args */)  // 构造函数 
{
}

Circular::~Circular()          // 析构函数
{
}