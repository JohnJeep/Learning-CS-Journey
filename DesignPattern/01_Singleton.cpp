/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2020-08-07 15:59:39
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单利模式为懒汉式模式，即在创建对象时，才分配内存
 * @FilePath: /01_Singleton.cpp
 */
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <mutex>


using namespace std;

class Singleton
{
private:
    static Singleton* spl;
    static mutex m_lock;

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


/**
 * @description: 懒汉式单例模式
 * @param {type} 
 * @return {type} 
 * @notice: 懒汉式或饿汉式单例模式在多个线程操作时，是不安全的。
 */
Singleton* Singleton::getInstance()
{
    if (spl == nullptr)
    {
        spl = new Singleton;     // 只有在创建对象时，才分配内存
    }
    return spl;
}

// 添加同步锁后保证了多个线程同时访问时安全的
Singleton* Singleton::getInstance()
{
    if (spl == nullptr)    // 双重检测机制
    {
        unique_lock<std::mutex> m_lock;  // 加同步锁，锁住整个类，防止new Singleton被执行多次
        {
            // 进入Synchronized 临界区以后，两个线程同时访问时，需要保证一个时间内只有一个线程在创建对象
            if (spl == nullptr)
            {
                spl = new Singleton;     
            }
        }
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



