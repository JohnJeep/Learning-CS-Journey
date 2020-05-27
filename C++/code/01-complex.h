/*
 * @Author: your name
 * @Date: 2020-05-27 15:10:18
 * @LastEditTime: 2020-05-27 16:16:53
 * @LastEditors: Please set LastEditors
 * @Description: // 声明相关的函数
 */ 
#ifndef  __01_COMPLEX_H
#define __01_COMPLEX_H

class complex
{
private:
    /* data */
    double re, im;   // 实部和虚部

    friend complex& __doapl (complex*, const complex&);  // __doapl()函数可以调用
    //complex& __doapl (complex* th , const complex& r)
public:
    // 构造函数
    complex (double r = 0.0, double i = 0.0)
    : re(r), im(i)         // 初始化数据
    {
        // 做其它的处理
        // 内存分配等等
    }

    // 对构造的函数要进行哪些操作？
    complex& operator += (const complex&);
    
    double real () const     // 实部和实部相加
    {
        return re;
    }
    double image () const   // 虚部和虚部相加
    {
        return im;
    }


    ~complex();
};

#endif // ! 