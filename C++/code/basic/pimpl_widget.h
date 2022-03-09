// 暴露在外部的头文件，Widget 类中并不清楚 Impl 类的实现，只知道接口

#ifndef _PIMPL_WIDGET_H
#define _PIMPL_WIDGET_H 
#include <memory>

// C++98风格
#if 0
class Widget
{
public:
    Widget(/* args */);
    ~Widget();

private:
    struct Impl;    // declare implementation struct
    Impl* PImpl;    // declare pointer
};
#endif

// C++11以上风格，支持智能指针
#if 1
class Widget
{
public:
    Widget(/* args */);
    ~Widget();

    // 支持 move operation
    Widget(Widget&& rhs);
    Widget& operator=(Widget&& rhs);

    // 支持拷贝赋值和拷贝构造，深拷贝
    Widget(const Widget& rhs);            // copy ctor
    Widget& operator=(const Widget& rhs); // copy operator=

private:
    struct Impl;                     // declare implementation struct
    std::unique_ptr<Impl> pImpl;     // declare pointer
};

#endif


#endif // _PIMPL_WIDGET_H