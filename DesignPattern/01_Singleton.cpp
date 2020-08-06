/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2020-08-06 23:08:58
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单利模式为懒汉式模式，即在创建对象时，才分配内存
 * @FilePath: /01_Singleton.cpp
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

class Singleton
{
private:
    static Singleton* spl;
    Singleton();
public:
    ~Singleton();
    static Singleton* getInstance();
    static Singleton* freeInstance();

};

Singleton* Singleton::spl = nullptr; // 静态全局变量初始化

Singleton::Singleton()
{
    cout << "执行构造函数" << endl;
}

Singleton::~Singleton()
{
    cout << "执行析构函数" << endl;
}

Singleton* Singleton::getInstance()
{
    if (spl == nullptr)
    {
        spl = new Singleton;     // 只有在创建对象时，才分配内存
    }
    return spl;
}

Singleton* Singleton::freeInstance()
{
    if (spl != nullptr)
    {
        delete spl;
        spl = nullptr;
    }
    cout << "释放对象内存" << endl;
    return spl;
}


int main(int argc, char *argv[])
{
    cout << "执行懒汉式的单例模式" << endl;

    Singleton *s1 = Singleton::getInstance();
    Singleton *s2 = Singleton::getInstance();
    if (s1 == s2)
    {
        cout << "是同一个对象" << endl;
    }
    else
    {
        cout << "不是同一个对象" << endl;
    }
    
    Singleton::freeInstance();

    return 0;
}



