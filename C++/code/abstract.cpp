/*
 * @Author: JohnJeep
 * @Date: 2020-06-10 09:26:56
 * @LastEditTime: 2020-06-10 11:26:36
 * @LastEditors: Please set LastEditors
 * @Description: 虚函数实现抽象类
 * @FilePath: /C++/code/abstract.cpp
 */ 
#include <iostream>
using namespace std;

class Shape
{
private:

public:
    Shape();                      // 父类无参数构造函数
    ~Shape();
    virtual void getArea() = 0;
};

Shape::Shape()
{
}

Shape::~Shape()
{
}


class Triangle:public Shape  // 子类继承父类
{
private:
    int ta;
    int tb;
public:
    Triangle(int a, int b); //显示调用父类的构造函数
    ~Triangle();
    virtual void getArea()
    {
        int area = (ta * tb) / 2;
        cout << "triangel area: " << area << endl;
    }
};

Triangle::Triangle(int a, int b)
:ta(a), tb(b)
{
}

Triangle::~Triangle()
{
}


class Rectangle:public Shape
{
private:
    int ra;
    int rb;
public:
    Rectangle(int a, int b);
    ~Rectangle();
    virtual void getArea()
    {
        int area = ra * rb;
        cout << "rectangle area: " << area << endl;
    }
};

Rectangle::Rectangle(int a, int b)
:ra(a), rb(b)
{

}

Rectangle::~Rectangle()
{
}

// 使用多态：父类指针执行子类对象
void calculateArea(Shape *obj)
{
    obj->getArea();
}

int main()
{
    Triangle t1(4, 6);
    Rectangle r1(4, 7);

    calculateArea(&t1);
    calculateArea(&r1);
    
    return 0;
}

