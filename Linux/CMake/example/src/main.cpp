/*
 * @Author: your name
 * @Date: 2021-05-16 11:12:00
 * @LastEditTime: 2022-05-12 01:55:07
 * @LastEditors: JohnJeep johnjeep1985@gmail.com
 * @Description: In User Settings Edit
 */
#include <iostream>
#include "integer.h"
using namespace std;

void TestInteger()
{
    Integer num1(1024);
    Integer num2(2048);
    Integer num3; 

    num3 = num1 + num2;
    cout << "进行加法重载运算的结果：" << num3.IntValue() << endl;
}

int main()
{
    TestInteger();

    return 0;
}
