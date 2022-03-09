/*
 * @Author: JohnJeep
 * @Date: 2022-02-15 23:20:18
 * @LastEditTime: 2022-02-16 00:51:48
 * @LastEditors: Please set LastEditors
 */
#include <iostream>

#if 0
// Widget 类依赖的额外头文件 
#include "pimpl_widget.h"
#include <cstring>      
#include <vector>
#include <gadget.h>

// C++98风格
struct Widget::Impl 
{
    std::string m_name;
    std::vector<double> m_data;
    // Gadget g1, g2, g3;        // 来自 gadget.h 文件
};

Widget::Widget()
    : pImpl(new Impl)      // 给 Widget 对象分配数据成员 
{
}

Widget::~Widget()
{
    delete pImpl;        // 释放 Widget 对象分配数据成员 
}
#endif


// C++11以上风格，支持智能指针
#if 1
// Widget 类依赖的额外头文件 

#include "pimpl_widget.h"
#include <cstring>      
#include <vector>
#include <gadget.h>

// C++98风格
struct Widget::Impl 
{
    std::string m_name;
    std::vector<double> m_data;
    // Gadget g1, g2, g3;        // 来自 gadget.h 文件
};

Widget::Widget()
    : pImpl(std::make_unique<Impl>())      // 给 Widget 对象分配数据成员 
{
}

// define dector
Widget::~Widget()
{
}


// 支持 move operation
Widget::Widget(Widget&& rhs) = default;
Widget::Widget::operator=(Widget&& rhs) = default;

// 支持拷贝赋值和拷贝构造，深拷贝
Widget::Widget(const Widget& rhs)
    : pImpl(std::make_unique<Impl>(*rhs.pImpl))
{
}

Widget& Widget::operator=(const Widget& rhs)
{
    *pImpl = *rhs.pImpl;
    return *this;
}


#endif

int main(int argc, char *argv[])
{
    Widget w1;
    auto w2(std::move(w1));    // w1 move-construct w2 
    w1 = std::move(w2);        // w2 move-assign to w1     
    return 0;
}
