/*
 * @Author: JohnJeep
 * @Date: 2020-09-09 15:08:06
 * @LastEditTime: 2020-09-09 16:17:31
 * @LastEditors: Please set LastEditors
 * @Description: 工厂模式实现
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

// 抽象的父类
class Factory
{
private:
    /* data */
public:
    Factory(/* args */){};
    virtual ~Factory(){};
    virtual Fruit* crate(string pstr) = 0;
};

// 具体的子类工厂
class AppleFactory : public Factory
{
private:
    /* data */
public:
    AppleFactory(/* args */) {}
    ~AppleFactory() {}
    virtual Fruit* crate(string pstr)
    {
        if (pstr == "apple")
        {
            return new Apple;
        }
        else
        {
            cout << "not crate." << endl;
        }
    }
};

// 扩展的产品
class MutatePear : public Fruit
{
private:
    /* data */
public:
    MutatePear(/* args */) {}
    ~MutatePear() {}
    virtual void getShape()
    {
        cout << "I am triangle." << endl;
    }
};

class MutateFactory : public Factory
{
private:
    /* data */
public:
    MutateFactory(/* args */) {}
    ~MutateFactory() {}
    virtual Fruit* crate(string str);
};

Fruit* MutateFactory::crate(string pstr)
{
    if (pstr == "Mutatepear")
    {
        return new MutatePear;
    }
    else
    {
        cout << "not crate." << endl;
    }  
}

int main(int argc, char *argv[])
{
    Factory *myfactory = nullptr;
    Fruit *myfruit = nullptr;

    myfactory = new AppleFactory;   // 父类对象指向子类对象
    myfruit = myfactory->crate("apple");
    myfruit->getShape();    // 使用多态，父类指针指向子类指针
    delete myfruit;
    delete myfactory;

    myfactory = new AppleFactory;
    myfruit = myfactory->crate("tomato");
    myfruit->getShape();    // 使用多态，父类指针指向子类指针
    delete myfruit;
    delete myfactory;

    myfactory = new MutateFactory;
    myfruit = myfactory->crate("Mutatepear");
    myfruit->getShape();
    delete myfruit;
    delete myfactory;

    return 0;
}