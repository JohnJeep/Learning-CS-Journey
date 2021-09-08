/*
 * @Author: JohnJeep
 * @Date: 2020-06-08 10:44:52
 * @LastEditTime: 2021-09-08 15:49:55
 * @LastEditors: Please set LastEditors
 * @Description: 派生类访问控制权限分析
 */
#include <iostream>
using namespace std;

class A {
private:
  int m_a;

protected:
  int m_b;

public:
  int m_c;

public:
  void set(int a, int b, int c) {
    cout << "set A class " << endl;
    this->m_a = a;
    this->m_b = b;
    this->m_c = c;
  }
  A(/* args */);
  ~A();
};

A::A(/* args */) {
  m_a = 0;
  m_b = 0;
  m_c = 0;
}

A::~A() {}

class B : public A {
private:
  /* data */
public:
  void showB() {
    cout << "show B class" << endl;
    // cout << "m_a:" << m_a << endl;  私有成员不能访问
    cout << "m_b:" << m_b << endl;
    cout << "m_c:" << m_c << endl;
  }
  B(/* args */);
  ~B();
};

B::B(/* args */) {}

B::~B() {}

class C : protected A {
private:
  /* data */
public:
  void showC() {
    cout << "show C class" << endl;
    // cout << "m_a:" << m_a << endl;  // private 私有成员不能访问
    cout << "m_b:" << m_b << endl; // protected
    cout << "m_c:" << m_c << endl; // protected
  }
  C(/* args */);
  ~C();
};

C::C(/* args */) {}

C::~C() {}

class D : private A {
private:
  /* data */
public:
  void showD() {
    cout << "show D class" << endl;
    // cout << "m_a:" << m_a << endl;  // private 私有成员不能访问
    cout << "m_b:" << m_b
         << endl; // 父类的成员变为 private属性，只能子类的内部使用
    cout << "m_c:" << m_c
         << endl; // 父类的成员变为 private属性，只能子类的内部使用
  }
  D(/* args */);
  ~D();
};

D::D(/* args */) {}

D::~D() {}

int main() {
  B b1;
  b1.set(11, 22, 33);
  b1.m_c = 100; // 类的外部能调用子类 public属性的成员变量
  b1.showB();

  C c1;
  //   c1.m_c = 200;  // 类的外部能调用子类 protected 属性的成员变量
  // c1.set(44, 55, 66);   // set() 函数属性为 protected，不能在类的外部使用
  c1.showC(); // 先调用 A类的构造函数，再调用子类（C类）的构造
              // 先调用子类（ C类）的析构函数，再调用父类（A类）的析构函数

  D d1;
  //   c1.m_c = 300; // 类的外部不能调用子类 private 属性的成员变量
  // d1.set();
  d1.showD();

  return 0;
}
