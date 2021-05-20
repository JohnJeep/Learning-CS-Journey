/*
 * @Author: JohnJeep
 * @Date: 2020-06-03 20:16:08
 * @LastEditTime: 2021-02-28 17:15:14
 * @LastEditors: Please set LastEditors
 * @Description: 构造函数、copy constructor、const类型变量初始化
 *               析构函数、组合对象混合等一起练习。
 *               
 */ 
#include <iostream>
using namespace std;

class Dog
{
public:
    Dog();                      // 无参构造
    Dog(int a, int b, int c);   // 有参构造
    ~Dog();

    // 其它成员函数
    int getOne()
    {
        return this->m_a;
    }

private:
   int m_a;
   int m_b;
   int m_c;
};

Dog::Dog()
{
    cout << "执行无参数构造" << endl;
}

Dog::Dog(int a, int b, int c)
{
    this->m_a = a;
    this->m_b = b;
    this->m_c = c;
    cout << "执行Dog类的三个参数构造函数：" 
         << this->m_a << ", "
         << this->m_b << ", "
         << this->m_c << ", "
         << endl;
}

Dog::~Dog()
{
    cout << "执行Dog类的析构函数: " 
         << this->m_a << ", "
         << this->m_b << ", "
         << this->m_c << ", "
         << endl;
}

/**
 * @brief: Bird类中组合了Dog类，并对Dog类进行初始化，创建了Bird类的copy constructor function
 */
class Bird
{
public:
    Bird();
    Bird(const Bird& parrot);
    ~Bird();

    Dog smallBlack;    // 对Bird类进行实例化，实例化对象为smallBlack
    Dog smallWhite;
    const int age;    // const类型变量在初始值列中完成初始化

private:
    int num;
};

Bird::Bird()
    : smallBlack(11, 22, 33), smallWhite(44, 55, 66), age(3)     // 初始化列表
{
    cout << "执行Bird类的构造函数" << endl;
}

Bird::Bird(const Bird& parrot)                                    // 拷贝构造函数
    : smallBlack(01, 02, 03), smallWhite(04, 05, 06), age(10)     // 初始化列表
{
    cout << "执行Bird类带有 const Bird& parrot 参数的构造函数" << endl;
}

Bird::~Bird()
{
    cout << "执行Bird类的析构函数" << endl;
}


// 调用函数
int doSomething(Bird pp)
{
    cout << "调用getOne()函数: " << pp.smallBlack.getOne() << endl;
    return 0;
}   


int main(int argc, char *argv[])
{
    Bird magpie;           // 默认去调用无参数的构造函数
    doSomething(magpie);   // 默认去调用有参数的构造函数

    cout << "\nExecute copy constructor" << endl;
    Bird bd = magpie;     // 调用copy constructor，将magpie对象中的数据copy给bd对象

    return 0;
}