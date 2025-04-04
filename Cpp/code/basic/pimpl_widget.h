#ifndef _PIMPL_WIDGET_H
#define _PIMPL_WIDGET_H 
#include <memory>

// C++98风格
#if 0
class Widget
{
public:
    Widget();
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
    Widget();
    ~Widget();   // only declare destructor

    // 支持 move operation，拷贝构造和assign操作声明
    Widget(Widget&& rhs);
    Widget& operator=(Widget&& rhs);

    // 支持拷贝赋值和拷贝构造，深拷贝
    Widget(const Widget& rhs);            // copy ctor
    Widget& operator=(const Widget& rhs); // copy operator=

private:
    struct Impl;                     // declare struct Impl
    std::unique_ptr<Impl> pImpl;     // declare pointer pImpl，建议用 std::unique_ptr 而不是 std::shared_ptr
};

#endif


#endif // _PIMPL_WIDGET_H