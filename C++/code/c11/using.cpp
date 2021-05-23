/*
 * @Author: JohnJeep
 * @Date: 2021-05-23 09:57:22
 * @LastEditTime: 2021-05-23 20:47:19
 * @LastEditors: Please set LastEditors
 * @Description: using 关键字的用法
 */
#include <iostream>
#include <string>
using namespace std;

class Parent
{
public:
    Parent(int i) : m_i(i) {}
    Parent(int i, double j) : m_i(i), m_j(j) {}
    Parent(int i, double j, string k) : m_i(i), m_j(j), m_k(k) {}

    void setI(int i) { m_i = i; }
    int getI() const { return m_i; }

    void setJ(int j) { m_j = j; }
    double getJ() const { return m_j; }

    void setK(string k) { m_k = k; }
    string getK() const { return m_k; }

private:
    int m_i;
    double m_j;
    string m_k;
};

class ChildOne : public Parent
{
public:
    ChildOne(int i) : Parent(i) {}
    ChildOne(int i, double j) : Parent(i, j) {}
    ChildOne(int i, double j, string k) : Parent(i, j, k) {}
};

class ChildTwo : Parent
{
private:
    /* data */
public:

    // 声明父类的构造函数，子类不需要定义相同的构造函数，而是在子类中直接调用父类的构造函数
    using Parent::Parent;

    // 声明子类继承与父类的成员函数
    using Parent::getI;
    using Parent::getJ;
    using Parent::getK;
};


int main()
{
    ChildOne c1(520, 13.14, "i love you");

    // 子类对象调用父类成员函数
    cout << "int: " << c1.getI() << "\n"
         << "double: " << c1.getJ() << "\n"
         << "string: " << c1.getK() << endl;

    ChildTwo c2(100, 66.6, "forever");
    cout << "int: " << c2.getI() << "\n"
         << "double: " << c2.getJ() << "\n"
         << "string: " << c2.getK() << endl;

    return 0;
}