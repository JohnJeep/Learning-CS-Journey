/*
 * @Author: JohnJeep
 * @Date: 2021-01-21 22:09:48
 * @LastEditTime: 2021-01-21 22:20:08
 * @LastEditors: Please set LastEditors
 * @Description: reference与pointer指针的区别
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

typedef struct Stag 
{
    int a, b, c, d;
} S;


int main(int argc, char *argv[])
{
    double x = 0;
    double* p = &x;
    double& r = x;

    cout << sizeof(x) << endl;  // 8 byte
    cout << sizeof(p) << endl;  // 根据机器是多少位决定的
    cout << sizeof(r) << endl;  // // 8 byte
    cout << p << endl;
    cout << *p << endl;
    cout << x << endl;
    cout << r << endl;
    cout << &x << endl;
    cout << &r << endl;

    S st;
    S& rs = st;
    cout << sizeof(st) << endl;
    cout << sizeof(rs) << endl;
    cout << &st << endl;
    cout << &rs << endl;

    return 0;
}
