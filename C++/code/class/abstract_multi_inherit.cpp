/*
 * @Author: your name
 * @Date: 2020-06-10 15:59:16
 * @LastEditTime: 2020-06-10 16:19:15
 * @LastEditors: Please set LastEditors
 * @Description: 抽象类中的多继承
 * @FilePath: /C++/code/abstract_multi_inherit.cpp
 */ 
#include <iostream>
using namespace std;

class Phone
{
private:
    /* data */
public:
    Phone(/* args */);
    ~Phone();
    virtual void camera() = 0;
    virtual void battery() = 0;
};

Phone::Phone(/* args */)
{
}

Phone::~Phone()
{
}

class HuaWei
{
private:
    /* data */
public:
    HuaWei(/* args */);
    ~HuaWei();
    virtual void baseBand() = 0;
};

HuaWei::HuaWei(/* args */)
{
}

HuaWei::~HuaWei()
{
}


class Xiaomi
{
private:
    /* data */
public:
    Xiaomi(/* args */);
    ~Xiaomi();
    virtual void surface() = 0;
};

Xiaomi::Xiaomi(/* args */)
{
}

Xiaomi::~Xiaomi()
{
}


class Oppo:public Phone, public HuaWei, public Xiaomi
{
private:
    /* data */
public:
    Oppo(/* args */);
    ~Oppo();

    virtual void camera()
    {
        cout << "继承Phone camera" << endl;
    }

    virtual void battery()
    {
        cout << "继承Phone battery" << endl;
    }

    void baseBand()
    {
        cout << "继承华为baseBand" << endl;
    }

    void surface()
    {
        cout << "继承小米界面" << endl;
    }
};

Oppo::Oppo(/* args */)
{
}

Oppo::~Oppo()
{
}


int main()
{
    Oppo o1;
    Phone *ph = &o1;   // 父类指针指向子类对象
    ph->battery();

    HuaWei& h1 = o1;   // 父类引用指向子类对象
    h1.baseBand();

    return 0;
}


