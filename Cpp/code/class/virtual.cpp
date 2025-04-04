/*
 * @Author: JohnJeep
 * @Date: 2020-06-09 08:49:18
 * @LastEditTime: 2020-06-09 09:42:41
 * @LastEditors: Please set LastEditors
 * @Description: 测试虚继承
 * @FilePath: \Learning-Computer\C++\code\virtual.cpp
 */ 
#include <iostream>
using namespace std;

class Flower
{
private:
    int size;
public:
    Flower(/* args */);
    ~Flower();
};

Flower::Flower(/* args */)
{
}

Flower::~Flower()
{
}


class Rose: public Flower
{
private:
    // int size;
public:
    Rose(/* args */);
    ~Rose();
};

Rose::Rose(/* args */)
{
}

Rose::~Rose()
{
}

// 使用virtual关键字C++编译器默认添加了一些属性给类
class WhiteRose: virtual public Flower  
{
private:
    // int size;
public:
    WhiteRose(/* args */);
    ~WhiteRose();
};

WhiteRose::WhiteRose(/* args */)
{
}

WhiteRose::~WhiteRose()
{
}

int main()
{
    cout << sizeof(Flower) << endl;
    cout << sizeof(Rose) << endl;
    cout << sizeof(WhiteRose) << endl;

    return 0;
}