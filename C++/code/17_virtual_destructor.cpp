/*
 * @Author: JohnJeep
 * @Date: 2020-06-09 11:56:19
 * @LastEditTime: 2020-06-09 14:22:58
 * @LastEditors: Please set LastEditors
 * @Description: 虚析构函数的实现
 * @FilePath: /C++/code/virtual_destructor.cpp
 */ 
#include <iostream>
#include <string.h>
using namespace std;

class Fish
{
private:
    char *color;
public:
    Fish(/* args */);
    virtual ~Fish();
};

Fish::Fish(/* args */)
{
    color = new char[10];
    strcpy(color, "black");
    cout << "fish color: " << color << endl;
}

Fish::~Fish()
{
    delete [] color;
    cout << "析构Fish" << endl;
}

class GoldFish : public Fish
{
private:
    char *color;
public:
    GoldFish(/* args */);
    virtual ~GoldFish();
};

GoldFish::GoldFish(/* args */)
{
    color = new char[10];
    strcpy(color, "gold");
    cout << "goldfish color: " << color << endl;

}

GoldFish::~GoldFish()
{
    delete [] color;
    cout << "析构GoldFish" << endl;
}

class CatFish : public GoldFish
{
private:
   char *color;
public:
    CatFish(/* args */);
    virtual ~CatFish();
};

CatFish::CatFish(/* args */)
{
    color = new char[10];
    strcpy(color, "gray");
    cout << "fish color: " << color << endl;
}

CatFish::~CatFish()
{
    delete [] color;
    cout << "析构CatFish" << endl;
}

/**
 * @description: 不加 virtual 关键字，只能释放父类对象的资源，不是多态类型，不能释放子类的资源
 * @param : 父类对象指针 
 * @return: null
 */
void releaseVirtual(Fish *obj)
{
    delete obj;   // 通过父类指针释放子类的资源
}


int main ()
{
    CatFish *cf = new CatFish;
    releaseVirtual(cf);
    // delete  cf;  // 不加virtual关键字释放父类与子类的资源，与使用多态方法实现的效果一样

    return 0;
}