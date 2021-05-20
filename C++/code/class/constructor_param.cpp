/*
 * @Author: JohnJeep
 * @Date: 2020-06-10 09:26:56
 * @LastEditTime: 2020-06-10 15:04:30
 * @LastEditors: Please set LastEditors
 * @Description: 子类继承父类中带有参数的构造函数
 * @FilePath: 
 */ 

#include <iostream>
using namespace std;

class Shape
{
public:
    int m_a;
    int m_b;
public:
    Shape();                        // 父类无参数构造函数
    // Shape(int x = 0, int y = 0);    // 父类带有幽默值参数的构造函数
    Shape(int x, int y);            // 父类参数没有默值的构造函数
    ~Shape();
    virtual void getArea() = 0;
};

Shape::Shape()
{
    cout << "调用默认构造" << endl;
}

Shape::Shape(int x, int y)
{
    this->m_a = x;
    this->m_b = y;
    cout << "Shape m_a: " << m_a << endl;
    cout << "Shape m_b: " << m_b << endl;
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
    // Triangle(int a, int b):Shape(100, 200) {} // 方法一：子类显示调用父类中参数没有默认值的构造函数
    Triangle(int a, int b); // 方法二：显示调用父类的构造函数，类中声明，构造函数实现时，指定默认值，只指定一次
    ~Triangle();
    virtual void getArea()
    {
        int area = (ta * tb) / 2;
        cout << "triangel area: " << area << endl;
    }
};

Triangle::Triangle(int a, int b):Shape(100, 200)
// :ta(a), tb(b)
{
    // 下面的语句与  :ta(a), tb(b) 等价
    this->ta = a;
    this->tb = b;
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

// 父类有参构造函数设置有默认值时，子类不能调用默认有参构造，设置为没有默认值时才可以调用
Rectangle::Rectangle(int a, int b)
:Shape()        // 初始化参数列表
{
    this->ra = a;
    this->rb = b;
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
    // Shape s1(11, 22);   //  error 抽象类不能实例化对象
    Triangle t1(4, 6);
    Rectangle r1(4, 7);

    calculateArea(&t1);
    calculateArea(&r1);
    
    return 0;
}

