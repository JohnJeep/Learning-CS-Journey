/*
 * @Author: JohnJeep
 * @Date: 2020-09-10 10:19:42
 * @LastEditTime: 2020-09-10 15:19:30
 * @LastEditors: Please set LastEditors
 * @Description: 抽象工厂模式理解
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Fruit
{
private:
    /* data */
public:
    Fruit() {}
    virtual ~Fruit() {}
    virtual void getName() = 0;
};

class SouthApple :public Fruit
{
private:
    /* data */
public:
    SouthApple() {}
    ~SouthApple() {}
    virtual void getName()
    {
        cout << "I am apple from south." << endl;
    }
};

class SouthPeach :public Fruit
{
private:
    /* data */
public:
    SouthPeach() {}
    ~SouthPeach() {}
    virtual void getName()
    {
        cout << "I am peach from south."  << endl;
    }
};

class NorthApple: public Fruit
{
private:
    /* data */
public:
    NorthApple() {}
    ~NorthApple() {}
     virtual void getName()
    {
        cout << "I am apple from north." << endl;
    }   
};

class NorthPeach : public Fruit
{
private:
    /* data */
public:
    NorthPeach() {}
    ~NorthPeach() {}
     virtual void getName()
    {
        cout << "I am peach from north." << endl;
    }   
};

class Factory
{
private:
    /* data */
public:
    Factory() {}
    virtual ~Factory() {}

    // 现在抽象的工厂类可以生产多个系列的产品
    virtual Fruit *produceApple() = 0;
    virtual Fruit *producePeach() = 0;
};

class SouthFactory : public Factory
{
private:
    /* data */
public:
    SouthFactory() {}
    ~SouthFactory() {}
    virtual Fruit *produceApple()
    {
        return new SouthApple;
    }

    virtual Fruit *producePeach()
    {
        return new SouthPeach;
    }
};

class NorthFactory : public Factory
{
private:
    /* data */
public:
    NorthFactory() {}
    ~NorthFactory() {}
    virtual Fruit *produceApple()
    {
        return new NorthApple;
    }
    virtual Fruit *producePeach()
    {
        return new NorthPeach;
    }
};

int main(int argc, char *argv[])
{
    Factory *myFactory = nullptr;
    Fruit *myFruit = nullptr;

    myFactory = new NorthFactory;
    myFruit = myFactory->produceApple();
    myFruit->getName();
    delete myFruit;
    myFruit = myFactory->producePeach();
    myFruit->getName();
    delete myFruit;
    delete myFactory;
    
    myFactory = new SouthFactory;
    myFruit = myFactory->produceApple();
    myFruit->getName();
    delete myFruit;
    myFruit = myFactory->producePeach();
    myFruit->getName();
    delete myFruit;
    delete myFactory;

    return 0;
}