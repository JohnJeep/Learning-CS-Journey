/*
 * @Author: JohnJeep
 * @Date: 2020-07-13 15:01:32
 * @LastEditTime: 2026-08-09 18:32:31
 * @LastEditors: JohnJeep
 * @Description: C++异常机制
 *
 */
#include <iostream>

class Stu {
 private:
  int m_id;
  int m_key;

 public:
  Stu(int id, int key);
  ~Stu();
};

Stu::Stu(int id, int key)
{
  this->m_id = id;
  this->m_key = key;
}

Stu::~Stu()
{
  std::cout << "执行析构函数" << "\n";
}

int divide(int x, int y)
{
  if (y == 0) {
    throw x;
  }

  return x / y;
}

// 测试用例1
void test01()
{
  try {
    int result = divide(10, 0);
    // int result = divide(10, 2);
    std::cout << "result: " << result << "\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  } catch (...) {
    std::cout << "test01未知异常" << "\n";
  }
}

// 测试用例2
void except()
{
  Stu wang(007, 100);
  std::cout << "执行异常" << "\n";
  throw 2;  // 抛出异常后类中的变量被析构了，内存空间被释放
}

void test02()
{
  try {
    except();
  } catch (int t) {
    std::cout << "int类型异常" << "\n";
  } catch (...) {
    std::cout << "test02未知异常" << "\n";
  }
}

int add(int x, int y)
{
  if (y == 0) {
    throw "y equal 0";
    // throw y;
  }

  return x / y;
}

// 处理普通的异常
void test03()
{
  try {
    int ret = add(4, 2);
    std::cout << "ret = " << ret << "\n";
    int re = add(4, 0);
    std::cout << "re = " << re << "\n";
  } catch (int e) {  // 捕获的类型由throw后面表达式的内容决定
    std::cout << e << "\n";
  } catch (const char* e) {
    std::cout << e << "\n";
  } catch (...) {
    std::cout << "execute ..." << "\n";
  }
}

// 在继承中使用异常
struct MyStruct : public std::exception {
  const char* what() const throw() { return "C++ exception"; }
};

void test04()
{
  try {
    throw MyStruct();
  } catch (MyStruct& e) {
    std::cout << "catch MyStruct" << "\n";

    // what() 是异常类提供的一个公共方法，它已被所有子异常类重载，这是返回异常产生的原因。
    std::cout << e.what() << "\n";
  }
}

int main(int argc, char* argv[])
{
  test01();
  test02();
  test03();
  test04();

  return 0;
}
