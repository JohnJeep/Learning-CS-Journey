/*
 * @Author: JohnJeep
 * @Date: 2020-07-13 10:49:35
 * @LastEditTime: 2020-07-13 14:17:39
 * @LastEditors: Please set LastEditors
 * @Description: C++类型转换
 * @FilePath: /C++/code/28_type_cast.cpp
 */ 
#include <iostream>
#include <cstdio>

using namespace std;

class Animal
{
private:
    int m_num;
public:
    Animal(/* args */);
    ~Animal();
    virtual void eat() = 0;// 虚函数
};

Animal::Animal(/* args */)
{
}

Animal::~Animal()
{
}

class Dog:public Animal
{
private:
    /* data */
public:
    Dog(/* args */);
    ~Dog();
    virtual void eat()
    {
        cout << "狗狗啃骨头" << endl;
    }    
    void dance()
    {
        cout << "狗狗跳舞！" << endl;
    }
};

Dog::Dog(/* args */)
{
}

Dog::~Dog()
{
}

class Cat:public Animal
{
private:
    /* data */
public:
    Cat(/* args */);
    ~Cat();
    virtual void eat()
    {
        cout << "小猫吃鱼！" << endl;
    }
    void sleep()
    {
        cout << "小猫要睡觉！" << endl;
    }
};

Cat::Cat(/* args */)
{
}

Cat::~Cat()
{
}

class Tiger
{
private:
    /* data */
public:
    Tiger(/* args */);
    ~Tiger();
};

Tiger::Tiger(/* args */)
{
}

Tiger::~Tiger()
{
}

// 使用多态的方式，父类指针指向子类的对象
void action(Animal *base)
{
    base->eat();
    Dog *pDog = dynamic_cast<Dog *>(base); // 动态类型转换
    if (pDog != nullptr)
    {
        pDog->dance();
    }
    
    Cat *pCat = dynamic_cast<Cat *>(base);
    if (pCat != nullptr)
    {
        pCat->sleep();
    }
    
}

void castChar(const char* p)
{ 
    if (p != nullptr)
    {
        char *p1 = nullptr;
        p1 = const_cast<char *>(p);  // 去除只读属性
        *(p1+1) = 'B';
        std::cout << "p: " << p << std::endl;
        std::cout << "p1: " << p1 << std::endl;
    }
}

int main(int argc, const char* argv[])
{
    double ret = 12.23;
    int pret = static_cast<int>(ret);
    std::cout << "static_cast: " << pret << std::endl;

    char *p1 = "this is string";
    int *p2 = nullptr;
    // p2 = static_cast<int*>(p1);   // 类型转换错误
    p2 = reinterpret_cast<int *>(p1);
    std::cout << "p1: " << p1 << std::endl;
    std::cout << "p2: " << p2 << std::endl;  // p2首地址

    Cat smallWhilte;
    Dog bigYellow;
    Tiger netherTiger;

    action(&smallWhilte);
    action(&bigYellow);
    
    Animal *pan = nullptr;
    pan = reinterpret_cast<Animal *>(&netherTiger);

    char pbuf[] = "abcdef";
    castChar(pbuf);

    return 0;
}