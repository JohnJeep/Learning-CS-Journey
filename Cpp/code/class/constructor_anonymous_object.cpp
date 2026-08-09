/*
 * @Author: JohnJeep
 * @Date: 2021-02-28 17:56:41
 * @LastEditTime: 2026-08-09 18:27:56
 * @LastEditors: JohnJeep
 * @Description: 构造函数中匿名对象的操作。匿名对象也叫临时对象(local object)。
 */
#include <cstdlib>
#include <iostream>
#include <string>

class Stu {
 public:
  Stu(std::string name, int id);  // constructor function
  Stu(const Stu& obj);            // copy constructor function
  ~Stu();                         // destructor function

  int getId() const
  {
    std::cout << "id: " << m_id << "\n";
    return m_id;
  }

  std::string getName() const
  {
    std::cout << "name: " << m_name << "\n";
    return m_name;
  }

 private:
  std::string m_name;
  int m_id;
};

Stu::Stu(std::string name, int id) : m_name(name), m_id(id)
{
  std::cout << "Execute constructor" << "\n";
}

Stu::~Stu()
{
  std::cout << "Execute destructor" << "\n";
  std::cout << "id: " << m_id << "\n";
  std::cout << "name: " << m_name << "\n";
}

Stu::Stu(const Stu& obj)
{
  m_id = obj.m_id;
  m_name = obj.m_name;
  std::cout << "Execute copy constructor" << "\n";
}

// 函数的返回值是一个对象
Stu func1()
{
  // local variable
  Stu tmp("wang", 007);
  return tmp;

  // return Stu("wang", 007);   // create local object, equal to above the sentence
}

int main(int argc, char* argv[])
{
  func1();  // 返回值是一个匿名对象，对象中的数据被析构

  // 用匿名对象去初始化 st 这个对象，C++编译器直接把匿名对象转化为新的有名对象，不被析构。
  Stu st = func1();
  st.getName();
  st.getId();

  // 用匿名对象去赋值给这个同类型的对象，匿名对象被析构。
  Stu art("li", 100);
  art = func1();  // art对象中原来的数据被func1返回对象中的数据覆盖

  return 0;
}
