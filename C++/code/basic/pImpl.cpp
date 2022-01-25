/*
 * @Author: JohnJeep
 * @Date: 2022-01-25 20:11:41
 * @LastEditTime: 2022-01-25 23:48:03
 * @LastEditors: Please set LastEditors
 * @Description: pImpl 实现
 */
#include <iostream>

using namespace std;

class A
{
private:
    /* data */
public:
    A(/* args */) {}
    ~A() {}

    void load()
    {
        cout << "load A ..." << endl;
    }
};

class B
{
private:
    /* data */
public:
    B(/* args */) {}
    ~B() {}

    void load()
    {
        cout << "load B ..." << endl;
    }
};

class ManagerA
{
private:
    A* pImpl;
public:
    ManagerA(/* args */) {}
    ~ManagerA() {}
    
    // load_A 接口暴露在外部的头文件中，A 类的具体实现接口不暴露
    void load_A()
    {
        pImpl->load();
    }
};

class ManagerB
{
private:
    B* pImpl;
    ManagerA MA;  // 组合了 ManagerA 类
public:
    ManagerB(/* args */) {}
    ~ManagerB() {}

    // load_B 接口暴露在外部的头文件中，B 类的具体实现接口不暴露
    void load_B()
    {
        pImpl->load();
    }

    // ManagerB 去调 ManagerA，ManagerA 中具体去调 A
    void B_A()
    {
        MA.load_A();
    }
};

// 暴露在外部的接口仅有 ManagerA、ManagerB 中的成员函数
int main(int argc, char *argv[])
{
    // ManagerA a;
    // a.load_A();

    ManagerB b;
    b.load_B();
    b.B_A();
    
    return 0;
}
