/*
 * @Author: JohnJeep
 * @Date: 2020-07-28 22:03:56
 * @LastEditTime: 2020-07-28 22:05:43
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式实现
 */ 
#include <iostream>
#include <stdlib.h>
#include <string>

using namespace std;

// 非线程安全，单线程中使用
class Singleton
{
private:
    Singleton(/* args */);   
    static Singleton* m_singleton;
    string m_value;

public:
    ~Singleton();

    static Singleton* getInstace();

    /**
     * Singletons should not be cloneable.
     */
    Singleton(Singleton& other) = delete;

    /**
     * Singletons should not be assignable.
     */
    void operator=(const Singleton&) = delete;

};

Singleton* Singleton::m_singleton = nullptr;

Singleton* Singleton::getInstace()
{
    if (m_singleton == nullptr) {
        m_singleton = new Singleton;
    }
    return m_singleton;
}


Singleton::Singleton(/* args */)
{
}

Singleton::~Singleton()
{
}


int main(int argc, char *argv[])
{
    
    return 0;
}