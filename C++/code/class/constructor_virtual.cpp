/*
 * @Author: JohnJeep
 * @Date: 2020-06-09 15:36:36
 * @LastEditTime: 2021-05-23 16:31:44
 * @LastEditors: Please set LastEditors
 * @Description: 构造函数中调用虚函数
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
  : m_id(100)
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
    show();    // 虚指针 vptr 分布在初始化时
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

