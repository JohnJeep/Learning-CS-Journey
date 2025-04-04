/*
 * @Author: JohnJeep
 * @Date: 2021-05-12 23:37:17
 * @LastEditTime: 2021-05-28 21:32:54
 * @LastEditors: Please set LastEditors
 * @Description: 模拟实现仿函数机制
 */
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <functional>

using namespace std;

template<typename T>
struct fplus
{
    T operator() (const T& x, const T& y) const {
        return x + y;
    }
};

template<typename T>
struct fminus
{
    T operator() (const T& x, const T& y) const {
        return x - y;
    }
};


using funcptr = void(*)(int, string);

class Fruit
{
public:
    string m_color;
    /* data */
public:
    Fruit() {}
    ~Fruit() {}
    
    void getValue(string name, int id) 
    {
        cout << "name: " << name << "\t" << "id: " << id << endl;
    }

    static void getAttr(int id, string color)
    {
        cout << "id: " << id << "\t" << "color: " << color << endl;
    }

    // 重载 operator() 操作符
    void operator()(int id) 
    {
        cout << "id: " << id << endl;
    }
     
    // 类对象转化为函数指针
    operator funcptr()
    {
        // static 修饰的成员函数属于类；没用 static 修饰的成员函数属于类的对象
        // getAttr 函数的返回类型与 函数指针 funcptr 的类型一致
        return getAttr;  
    }
};

// 重载 operator() 操作运算符，来作为可调用对象
void test01()
{
    fplus<int> plus_obj;
    fminus<int> minus_obj;

    cout << plus_obj(3, 5) << endl;
    cout << minus_obj(10, 3) <<endl;

    cout << fplus<int>()(5, 1) << endl;
    cout << fminus<int>()(100, 2) << endl;
    
    return;
}


void test02()
{
    // 重载 operator() 
    Fruit apple;
    apple(3);

    // 类对象转化为函数指针
    Fruit banana;
    banana(4, "yellow");

    // 类的普通成员函数指针
    using fptr = void(Fruit::*)(string name, int id);
    fptr fp = Fruit::getValue;
    Fruit peach;
    (peach.*fp)("peach", 100);   // * 优先级低于 () 优先级
    
    // 类的成员变量指针
    using mptr = string Fruit::*;   // 定义类成员变量函数指针
    mptr mp = &Fruit::m_color;
    peach.*mp = "orange";      // 类成员赋处值
    cout << "color: " << peach.m_color << endl;
}

int main(int argc, char *argv[])
{
    // test01();
    test02();

    return 0;
}