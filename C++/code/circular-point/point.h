/*
 * @Author: JohnJeep
 * @Date: 2020-06-01 14:09:46
 * @LastEditTime: 2020-06-01 15:51:41
 * @LastEditors: Please set LastEditors
 * @Description: 点类的声明
 */ 
#ifndef __POINT_H
#define __POINT_H
#include <iostream>
using namespace std;
class Point
{
private:
    int x1;
    int y1;
public:
    void setPoint(int px0, int py0);
    int getPointX1();
    int getPointY1();

    Point(/* args */);
    ~Point();
};

Point::Point(/* args */)
{
}

Point::~Point()
{
}

#endif // !__POINT_H



