/*
 * @Author: JohnJeep
 * @Date: 2020-06-08 14:39:31
 * @LastEditTime: 2020-06-08 15:00:39
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */ 
#include <iostream>
using namespace std;

class Parent
{
private:
    /* data */
public:
    Parent(/* args */);
    ~Parent();
    void showParent()
    {
        cout << "I am parent class" << endl;
    }
};

Parent::Parent(/* args */)
{
}

Parent::~Parent()
{
}

class Child: public Parent
{
private:
    /* data */
public:
    Child(/* args */);
    ~Child();
    void showChild()
    {
        cout << "I am child class" << endl;
    }
};

Child::Child(/* args */)
{
}

Child::~Child()
{
}

void display(Parent *base)
{
    cout << "指针做函数参数" << endl;
}

void displayRef(Parent& base)
{
    cout << "引用做函数参数" << endl;
}

int main()
{
    Child chd;

    Parent pat = chd;   // 子类对象直接初始化父类对象
    chd.showChild();
    chd.showParent();
    pat.showParent();

    Parent *p1 = NULL;
    p1 = &chd;    // 基类（父类）的指针或引用直接指向子类的对象。
    p1->showParent();

    // 指针做函数参数
    display(&pat);
    display(&chd);  // 基类（父类）的指针或引用直接指向子类的对象。

    // 引用做函数参数
    displayRef(pat);
    displayRef(chd); // 基类（父类）的指针或引用直接指向子类的对象。

    system("pause");
    return 0;
}