/*
 * @Author: JohnJeep
 * @Date: 2020-05-27 15:10:18
 * @LastEditTime: 2020-05-27 23:18:21
 * @LastEditors: Please set LastEditors
 * @Description: 模拟C++标准库中的复数函数的实现。
 */
#include <iostream>
#include "01-complex.h"

using namespace std;

// 操作符输出重定向
ostream& operator<<(ostream& os, const complex& x)
{
    return os << "(" << real(x) << "," << image(x) << ")";
}

int main()
{
    complex c1(2, 3);
    complex c2(4, 5);

    cout<< c1 << endl;
    cout<< (c1 += c2) << endl;
    return 0;
}
