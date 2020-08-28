/*
 * @Author: JohnJeep
 * @Date: 2020-06-05 08:30:53
 * @LastEditTime: 2020-08-28 15:22:53
 * @LastEditors: Please set LastEditors
 * @Description: 友元关键字例子：友元函数和友元类
 */ 
#include <iostream>
using namespace std;

class Pen
{
private:
    int m_a;
    int m_b;
    friend class Book;    // Book是pen的友元类，在Book类中可以调用Pen类中的属性和方法
public:
    Pen(int s, int t);
    ~Pen();
    friend void write(Pen *wt, int a);           // 友元函数
    int getA();
    int getB();
};

Pen::Pen(int s = 0, int t = 0) :m_a(s), m_b(t)
{
    this->m_a = s;
    this->m_b = t;
}

Pen::~Pen()
{
    std::cout << "默认执行析构函数" << std::endl;
}

/**
 * @description: 友元函数：外部函数去访问类内部的成员变量
 * @param:       类的指针 
 * @return:      null
 */
void write(Pen *wt, int a)
{
    wt->m_a = a;
    cout << "pen class call write function." << endl;
}

int Pen::getA()  // 成员函数
{
    return this->m_a;   
}

int Pen::getB()  // 成员函数
{
    return this->m_b;
}

class Book
{
private:
    Pen painting;         // 友元类 
public:
    void getB(int m);
    void show();

    Book();
    ~Book();
};

void Book::getB(int m)
{
    painting.m_b = m;
    cout << "get b:" << painting.m_b << endl;
}

void Book::show()
{
    int tmp = painting.getB();
    cout << "show tmp:" << tmp <<endl;
}

Book::Book()
{
}

Book::~Book()
{
}


int main(int argc, char *argv[])
{
    Pen pencil(11, 22);
    cout << "get digital: " << pencil.getA() << endl;
    write(&pencil, 100);
    cout << "get digital: " << pencil.getA() << endl;

    // 调用成员类
    Book recie;
    recie.getB(200);
    recie.show();
    
    return 0;
}