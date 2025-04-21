/*
 * @Author: JohnJeep
 * @Date: 2020-09-09 15:08:06
 * @LastEditTime: 2020-09-09 16:02:10
 * @LastEditors: Please set LastEditors
 * @Description: 简单工厂模式，不属于标准的工厂模式，不符合开闭原则，但用的比较多。
 * 
 */
#include <iostream>
#include <cstdio>
#include <cstring>

using namespace std;

// 父类为抽象的类
class Fruit   
{
private:
    /* data */
public:
    Fruit(/* args */) {}
    virtual ~Fruit() {}
    virtual void getShape() = 0;
};

class Apple : public Fruit
{
private:
    /* data */
public:
    Apple(/* args */) {}
    ~Apple() {}
    void getShape()
    {
        cout << "I am round." << endl;
    }
};

class Pear : public Fruit
{
private:
    /* data */
public:
    Pear(/* args */) {}
    ~Pear() {}
    void getShape()
    {
        cout << "I am rectangle." << endl;
    }
};

class Factory
{
private:
    /* data */
public:
    Factory(/* args */);
    ~Factory();
    Fruit* produce(string pstr);
};

Factory::Factory(/* args */)
{
}

Factory::~Factory()
{
}

Fruit* Factory::produce(string pstr)
{
    if (pstr == "apple")
    {
        return new Apple;
    }
    else if (pstr == "pear")
    {
        return new Pear;
    }
    else
    {
        cout << "not produce." << endl;
    }  
}

int main(int argc, char *argv[])
{
    Factory *myfactory = new Factory;
    Fruit *myfruit = nullptr;

    myfruit = myfactory->produce("apple");
    myfruit->getShape();    // 使用多态，父类指针指向子类指针
    delete myfruit;


    myfruit = myfactory->produce("pear");
    myfruit->getShape();
    delete myfruit;
    delete myfactory;

    return 0;
}