/*
 * @Author: JohnJeep
 * @Date: 2022-02-15 23:20:18
 * @LastEditTime: 2023-04-01 09:40:48
 * @LastEditors: Please set LastEditors
 */

/**
 * @brief 采用 Pimpl 模式，减少编译时不必要的依赖，减少暴露在外部头文件中的内容
 *        头文件像#include <string> #include <vector>，这些放在 Widget 类的头文件就不合适，
 *        因为并不想第三方知道Widget的实现，只让他知道接口就可以了。
 * 
 *        另外的原因：若将一些头文件放在 Widget 类的头文件中，只要头文件发生了变化，Widget 类的
 *        所有内容，都需要重新编译，增加了编译时间。
 *        例如：一个 Widget 类包含了另一个类 Impl，但 Widget 中并不清楚 Impl 类的实现，只知道接口
 * 
 */
#include <iostream>

#if 0
// Widget 类依赖的额外头文件 
#include "pimpl_widget.h"
#include <string>      
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
    : pImpl(new Impl)      // 创建 pImpl 对象
{
}

Widget::~Widget()
{
    delete pImpl;        // 释放 pImpl 对象
}
#endif


// C++11以上风格，支持智能指针
#if 1
// Widget 类依赖的额外头文件 

#include "pimpl_widget.h"
#include <string>      
#include <vector>
// #include <gadget.h>

// definition Impl 类的成员变量
struct Widget::Impl 
{
    std::string m_name;
    std::vector<double> m_data;
    // Gadget g1, g2, g3;        // 来自 gadget.h 文件
};

// std::make_unique 是 C++14 才支持的特性
Widget::Widget()
    : pImpl(std::make_unique<Impl>())      // 给 Widget 对象分配数据成员 
{
}

// 析构函数是必要的，要不然编译器执行默认的析构函数，去析构 std::unique_ptr<Impl> 的成员
// 变量，而 std::unique_ptr<Impl> 内部使用默认的删除器（deleter），deleter 是一个函数，
// 在 std::unique_ptr 内部，它使用 delete 关键字去删除一个原生的指针。
// 但是，在 delete 之前，通常默认的 deleter 使用 C++11 static _ assert 
// 来确保原始指针不指向不完整的类型。 这会导致显式创建了一个对象，而对象的销毁是隐式的。
// definition dector
Widget::~Widget()
{
}

// 支持 move operation，此处为 definition
// default 关键字只能放在 .cpp 中，不能放在头文件中，因为头文件中的
// Pimpl 对象是一个不完整的类型
Widget::Widget(Widget&& rhs) = default;
Widget& Widget::operator=(Widget&& rhs) = default;

// 拷贝构造，深拷贝
Widget::Widget(const Widget& rhs)
    : pImpl(std::make_unique<Impl>(*rhs.pImpl))
{
}

// 拷贝赋值
// Widget 的 operation= 操作实际上是调用编译器为类 Impl 生成的拷贝赋值
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
