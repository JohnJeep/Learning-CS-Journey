/*
 * @Author: JohnJee
 * @Date: 2021-05-27 22:47:59
 * @LastEditTime: 2021-09-09 22:34:05
 * @LastEditors: Windows10
 * @Description: 构造函数、拷贝构造函数、拷贝赋值函数区别
 */
#include <iostream>
#include <string>

using namespace std;

class Pen
{
private:
  /* data */
public:
  Pen(/* args */) { cout << "default construct..." << endl; }

  Pen(const Pen& p) { cout << "copy constructor..." << endl; }

  Pen& operator=(const Pen& _p) { return *this; }

  void show() { cout << "showing..." << endl; }

  ~Pen() { cout << "deconstructor..." << endl; }
};

int main(int argc, char* argv[])
{
  Pen();  // 创建临时对象，执行默认构造函数
  Pen t1; // stack 上实例化一个对象 t1，并分配内存空间
  t1.show();

  Pen t2(t1); // 调用拷贝构造函数，并实例化一个对象 t2
  t2.show();

  Pen
  t3(); // stack 上没有实例化对象，只是声明了一个对象，哪个地方用时，编译器再分配内存空间
  // t3 = t1;      // error

  Pen t4;
  t4 = t1; // 调用拷贝赋值

  return 0;
}