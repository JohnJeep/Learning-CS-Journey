/*
 * @Author: your name
 * @Date: 2020-06-01 14:09:38
 * @LastEditTime: 2020-06-01 15:10:35
 * @LastEditors: Please set LastEditors
 * @Description: 点类的实现
 */ 
#include "point.h"
#include "circular.h"

void Point::setPoint(int px0, int py0)
{
    x1 = px0;
    y1 = py0;
}

int Point::getPointX1() 
{
    return x1;
}

int Point::getPointY1() 
{
    return y1;
}
