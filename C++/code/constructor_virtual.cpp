/*
 * @Author: JohnJeep
 * @Date: 2020-06-09 15:36:36
 * @LastEditTime: 2020-06-09 16:04:09
 * @LastEditors: Please set LastEditors
 * @Description: 构造函数中调用虚函数
 * @FilePath: /C++/code/constructor_virtual.cpp
 */ 
#include <iostream>
using namespace std;

class Parent
{
private:
    int m_id;
public:
    Parent(/* args */);
    ~Parent();
    virtual void show()
    {
        cout << "parent m_id: "<< m_id << endl;
    }
};

Parent::Parent(/* args */)
:m_id(100)
{
    show();  // 构造函数中调用虚函数
}

Parent::~Parent()
{
}


class Child : public Parent
{
private:
    int m_id;
public:
    Child(/* args */);
    ~Child();
    virtual void show()
    {
        cout << "child m_id: "<< m_id << endl;
    }
};

Child::Child(/* args */)
:m_id(20)
{
    show();    // 虚指针 vptr 是分布初始化完成的。
}

Child::~Child()
{
}


int main()
{
    Child lucy;
    lucy.show(); 

    return 0;
}

