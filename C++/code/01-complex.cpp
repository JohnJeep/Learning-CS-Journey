/*
 * @Author: JohnJeep
 * @Date: 2020-05-27 15:10:18
 * @LastEditTime: 2020-05-27 16:18:02
 * @LastEditors: Please set LastEditors
 * @Description: 模拟C++标准库中的复数函数的实现。
 */ 
#include <iostream>
#include "01-complex.h"

int main ()
{
    inline complex& __doapl (complex* th , const complex& r)  // 定义__doapl()函数
    {
        // 进行加运算
        th->re += r.re; 
        th->im += r.im; 
        return *th;
    }

    inline complex& complex ::operator+= (const complex& r) // 定义重载操作符运算函数
    {
        return __doapl (this, r);
    }

    // 函数返回值不是传递的引用
    inline complex operator+ (const complex& x, const complex& y)  // 怎样去加操作
    {
        // 返回值为临时对象
        return complex (real (x) + real (y),        // 实部和实部相加
                    (image (x), image (y));

    } 

    inline complex operator+ (cosnt complex& x, double y)  // 实部不能变
    {
        return complex (real (x) + y, image (x));
    }

    inline complex operator+ (double x, cosnt complex& y)
    {
        return complex (x + image (y), image (y));
    }

    // 操作符输出重定向
    ostream& operator<< (ostream& os, const complex& x)
    {
        return os << "(" << real (x) << "," << image (y) << ")";

    }

    return 0;
}

















