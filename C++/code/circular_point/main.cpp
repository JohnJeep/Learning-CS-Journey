/*
 * @Author: JohnJeep
 * @Date: 2020-06-01 14:08:38
 * @LastEditTime: 2020-06-01 19:57:28
 * @LastEditors: Please set LastEditors
 * @Description: 面向对象实现判断点在圆外还是在圆内？
 */ 
#include <iostream>
#include "circular.h"
#include "point.h"
using namespace std;

int main(int argc, char* argv[])
{
    Circular c1;
    Point p1;

    c1.setCircle(2, 3, 3);
    p1.setPoint(7, 7);

    int tag = c1.judgePosition(p1);
    if (tag == 1)
    {
        cout << "point in circle" << endl;
    }
    else
    {
        cout << "point out circle" << endl;
    }
    
    return 0;
}















