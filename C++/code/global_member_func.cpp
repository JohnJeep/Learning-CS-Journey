/*
 * @Author: JohnJeep
 * @Date: 2020-06-04 14:03:43
 * @LastEditTime: 2020-06-04 15:57:09
 * @LastEditors: Please set LastEditors
 * @Description: 全局函数与成员函数
 */ 
#include <iostream>
using namespace std;

class Fruit
{
private:
    int m_a;
    int m_b;
    int m_c;

public:
    void show();
    Fruit memberAdd(Fruit& t1);
    Fruit& add(Fruit& s1);

    Fruit(int a, int b);
    ~Fruit();
};

/**
 * @description: 直接调用成员函数，显示成员变量的值
 * @param {type} 
 * @return: 
 */
void Fruit::show()
{
    cout << "m_a="<< m_a << ", " << "m_b=" << m_b << endl;
}

/**
 * @description: 将成员函数转成全局函数，不需要引入 this 指针
 * @param {type} 成员变量必须为 public 类型的属性，不能为private
 * @return: 
 */
// void show(Fruit *s)
// {
//     cout << "m_a=" << s->m_a << "m_b=" << s->m_b <<endl;
// }

/**
 * @description: 成员函数，实现两个对象相加。将全局函数变为类的成员函数，需要引入 this 指针
 * @param :      传入形参的对象 t1
 * @return:      返回结果是一个 Fruit 类的对象
 */
Fruit Fruit::memberAdd(Fruit& t1)    
{
    Fruit tmp(this->m_a + t1.m_a, this->m_b + t1.m_b);    // 会调用构造函数
    return tmp;      // tmp是一个对象
}

/**
 * @description: 成员函数，实现两个对象相加。
 * @param {type} 
 * @return:       函数返回的结果为类的引用 
 */
Fruit& Fruit::add(Fruit& s1)
{
    this->m_a = this->m_a + s1.m_a;
    this->m_b = this->m_b + s1.m_b;
    return *this;
}

/**
 * @description: 有参数构造函数
 * @param {type} 
 * @return: 
 */
Fruit::Fruit(int a=1, int b=1)    
:m_a(a), m_b(b)
{
    this->m_a = a;
    this->m_b = b;
    cout << "执行构造函数: " << m_a << ", " << m_b <<endl;
}

/**
 * @description: 默认调用析构函数
 * @param {type} 
 * @return: 
 */
Fruit::~Fruit()
{
    cout << "执行析构函数" << endl;
}

/**
 * @description: 定义全局函数
 * @param:       类的不同对象
 * @return:      类的目标对象
 */
Fruit globalAdd(Fruit& t1, Fruit& t2)
{
    Fruit tmp;
    // tmp = t1 + t2;  // 两个对象之间相加，需要重载操作符

    return tmp;
}


int main(int argc, char *argv[])
{
    Fruit strawberry;
    Fruit apple(11, 22); 
    Fruit banana(33, 44); 
    cout << endl;

    Fruit g_mix = globalAdd(apple, banana);                    // 调用全局函数
    cout << endl;
    
    Fruit mix = apple.memberAdd(banana);   // 匿名对象直接转化为 mix 对象；
                                           // 将banana对象中的变量与apple对象中的变量相加后赋值给apple对象中的变量
    mix.show();
    // show(&mix);    // 全局方式调用成员函数

    // 匿名对象赋值方式，使用等号的方法
    // Fruit mix_t;
    // mix_t = apple.memberAdd(banana);
    
    // 采用引用的方式调用
    cout << endl << "采用引用的方式调用" << endl;
    Fruit& mix_f = apple.add(banana);
    mix_f.show();

    system("pause");
    return 0;
}

