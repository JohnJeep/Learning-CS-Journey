/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 11:39:23
 * @LastEditTime: 2020-12-24 22:59:20
 * @LastEditors: Please set LastEditors
 * @Description: 模板模式：父类的抽象类定义方法，在子类中去实现
 * 
  */
#include <cstdio>
#include <iostream>

using namespace std;

class AbstactAnimal
{
private:
    /* data */
public:
    AbstactAnimal(/* args */) {}
    virtual ~AbstactAnimal() {}

    // 可变的部分，需要子类中实现
    virtual void flying() = 0;
    virtual void running() = 0;

    /**
   * The template method defines the skeleton of an algorithm.
   *  stable part.
   */
    void action() // 模板函数把业务结点确定好了
    {
        flying();
        running();
    }
};

class Bird : public AbstactAnimal
{
private:
    /* data */
public:
    Bird(/* args */) {}
    ~Bird() {}

    virtual void flying()
    {
        cout << "I am bird, I am flying" << endl;
    }
    virtual void running()
    {
        cout << "I am bird, I am running" << endl;
    }
};

class Mammal : public AbstactAnimal
{
private:
    /* data */
public:
    Mammal(/* args */) {}
    ~Mammal() {}

    virtual void running()
    {
        cout << "I am lion, I am running" << endl;
    }
    virtual void flying()
    {
        cout << "I am lion, I am flying" << endl;
    }
};

int main(int argc, char *argv[])
{
    AbstactAnimal *parrot = new Bird;
    parrot->action(); // 直接调用抽象类中的模板函数
    delete parrot;

    AbstactAnimal *lion = new Mammal;
    lion->action();
    delete lion;

    return 0;
}