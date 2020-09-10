/*
 * @Author: JohnJeep
 * @Date: 2020-09-09 15:08:06
 * @LastEditTime: 2020-09-10 11:36:10
 * @LastEditors: Please set LastEditors
 * @Description: 工厂模式实现
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
    Fruit() {}
    virtual ~Fruit() {}
    virtual void getShape() = 0;
};

class Apple : public Fruit
{
private:
    /* data */
public:
    Apple() {}
    ~Apple() {}
    void getShape()
    {
        cout << "Apple is circular." << endl;
    }
};

class BadFruit
{
private:
    /* data */
public:
    BadFruit(/* args */) {cout << "bad fruit." << endl;}
    ~BadFruit() {}
};

// 抽象的父类
class Factory
{
private:
    /* data */
public:
    Factory(){};
    virtual ~Factory(){};

    // 抽象的工厂类只能生产一个系列的产品
    virtual Fruit* produce(string pstr) = 0;
};

// 具体的子类工厂
class AppleFactory : public Factory
{
private:
    /* data */
public:
    AppleFactory() {}
    ~AppleFactory() {}
    virtual Fruit* produce(string pstr)
    {
        if (pstr == "apple")
        {
            return new Apple;
        }
        else
        {
            throw "Not apple not produce.";
        }
    }
};

// 扩展的产品
class MutatePear : public Fruit
{
private:
    /* data */
public:
    MutatePear() {}
    ~MutatePear() {}
    virtual void getShape()
    {
        cout << "MutatePear is triangle." << endl;
    }
};

class MutateFactory : public Factory
{
private:
    /* data */
public:
    MutateFactory() {}
    ~MutateFactory() {}
    virtual Fruit* produce(string pstr);
};

Fruit* MutateFactory::produce(string pstr)
{
    if (pstr == "Mutatepear")
    {
        return new MutatePear;
    }
    else
    {
        throw "Not mutatepear not produce.";
    }  
}

int main(int argc, char *argv[])
{
    Factory *myFactory = nullptr;   // 定义Factory抽象类的指针，但不能实例化对象
    Fruit *myFruit = nullptr;

    try
    {
        myFactory = new AppleFactory;   // 父类对象指向子类对象
        myFruit = myFactory->produce("apple");
        myFruit->getShape();    // 使用多态，父类指针指向子类指针
        delete myFruit;
        delete myFactory;

        myFactory = new MutateFactory;
        myFruit = myFactory->produce("Mutatepear");
        myFruit->getShape();
        delete myFruit;
        delete myFactory;

        myFactory = new AppleFactory;
        myFruit = myFactory->produce("tomato");
        myFruit->getShape();    // 使用多态，父类指针指向子类指针
        delete myFruit;
        delete myFactory;        
    }
    catch(const char* e)
    {
        cout << e << endl;
    }

    return 0;
}