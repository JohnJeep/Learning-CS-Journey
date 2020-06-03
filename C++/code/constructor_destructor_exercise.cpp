/*
 * @Author: your name
 * @Date: 2020-06-03 20:16:08
 * @LastEditTime: 2020-06-03 21:16:34
 * @LastEditors: Please set LastEditors
 * @Description: 构造函数、析构函数、组合对象混合一起练习。
 */ 

#include <iostream>
using namespace std;


class Dog
{
private:
   int m_a;
   int m_b;
   int m_c;

public:
    Dog();                      // 无参构造
    Dog(int a, int b, int c);   // 有参构造
    ~Dog();

    // 其它成员函数
    int getOne()
    {
        return this->m_a;
    }
};

class Bird
{
private:
    int num;

public:
    Bird();
    Bird(const Bird& parrot);
    ~Bird();

    Dog smallBlack;    // 在Bird类中实例化一个对象
    Dog smallWhite;
    const int age;
};

Bird::Bird()
: smallBlack(11, 22, 33), smallWhite(44, 55, 66), age(3)     // 初始化列表
{
    cout << "执行Bird类的构造函数" << endl;
}

Bird::Bird(const Bird& parrot)
: smallBlack(01, 02, 03), smallWhite(04, 05, 06), age(10)     // 初始化列表
{
    cout << "执行Bird类带有 const Bird& parrot 参数的构造函数" << endl;
}

Bird::~Bird()
{
    cout << "执行Bird类的析构函数" << endl;
}



Dog::Dog(/* args */)
{
    cout << "执行无参数构造" << endl;
}

Dog::Dog(int a, int b, int c)
{
    this->m_a = a;
    this->m_b = b;
    this->m_c = c;
    cout << "执行三个参数构造函数：" 
         << this->m_a
         << this->m_b
         << this->m_c
         << endl;
}

Dog::~Dog()
{
    cout << "执行析构函数" 
         << this->m_a
         << this->m_b
         << this->m_c
         << endl;
}


// 调用函数
int doSomething(Bird pp)
{
    cout << "调用getOne()函数" << pp.smallBlack.getOne() << endl;
    return 0;
}   



int main(int argc, char *argv[])
{
    Bird magpie;
    doSomething(magpie); 

    return 0;
}





