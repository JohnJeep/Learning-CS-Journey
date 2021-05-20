/*
 * @Author: your name
 * @Date: 2020-05-27 15:10:18
 * @LastEditTime: 2020-05-28 09:08:09
 * @LastEditors: Please set LastEditors
 * @Description: // 声明相关的函数
 */
#ifndef __01_COMPLEX_H
#define __01_COMPLEX_H
class complex;
complex& __doapl(complex *th, const complex& r); // 定义__doapl()函数

class complex
{
private:
    /* data */
    double re, im; // 实部和虚部

    friend complex& __doapl(complex *, const complex&); // __doapl()函数可以调用

public:
    // 构造函数
    complex(double r = 0.0, double i = 0.0)
        : re(r), im(i) // 初始化数据
    {
        // 做其它的处理
        // 内存分配等等
    }

    // 对构造的函数要进行哪些操作？
    complex& operator+=(const complex&);

    double real() const // 实部和实部相加
    {
        return re;
    }
    double image() const // 虚部和虚部相加
    {
        return im;
    }
};

inline complex& __doapl(complex *th, const complex& r) // 定义__doapl()函数
{
    // 进行加运算
    th->re += r.re;
    th->im += r.im;
    return *th;
}

inline complex& complex ::operator+=(const complex& r) // 定义重载操作符运算函数
{
    return __doapl(this, r);
}


inline double real(const complex& x)
{
    return x.real();
}

inline double image(const complex& y)
{
    return y.image();
}


// 函数返回值不是传递的引用
inline complex operator+(const complex& x, const complex& y) // 怎样去加操作
{
    // 返回值为临时对象
    return complex(real(x) + real(y), image(x) + image(y)); // 实部和实部相加
}

inline complex operator+(const complex& x, double y) // 实部不能变
{
    return complex(real(x) + y, image(x));
}

inline complex operator+(double x, const complex& y)
{
    return complex(x + image(y), image(y));
}

#endif