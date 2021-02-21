/*
 * @Author: JohnJeep
 * @Date: 2021-02-20 17:14:47
 * @LastEditTime: 2021-02-21 11:35:32
 * @LastEditors: Please set LastEditors
 * @Description: 单利模式单线程实现
 */
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace std;

// 非线程安全，单线程中使用
class Singleton
{
private:
    Singleton(const std::string value);   
    static Singleton* m_singleton;
    std::string m_value;

public:
    ~Singleton();

    static Singleton* getInstace(const std::string& value);

    /**
     * Singletons should not be cloneable.
     */
    Singleton(Singleton& other) = delete;

    /**
     * Singletons should not be assignable.
     */
    void operator=(const Singleton&) = delete;
    
    std::string getValue() 
    {
        return m_value;
    }
};

Singleton* Singleton::m_singleton = nullptr;


// 没有加锁，存在多个线程访问时资源竞争的问题
Singleton* Singleton::getInstace(const std::string& value)
{
    if (m_singleton == nullptr) {
        m_singleton = new Singleton(value);
    }
    return m_singleton;
}


Singleton::Singleton(const std::string value)
    : m_value(value)
{
}

Singleton::~Singleton()
{
}

void ThreadFirst()
{
    std::this_thread::sleep_for(chrono::milliseconds(1000));
    Singleton* singleton = Singleton::getInstace("First");
    std::cout << singleton->getValue() << std::endl;
}

void ThreadSecond()
{
    std::this_thread::sleep_for(chrono::milliseconds(1000));
    Singleton* singleton = Singleton::getInstace("Second");
    std::cout << singleton->getValue() << std::endl;
}

int main(int argc, char *argv[]) 
{
    std::thread t1(ThreadFirst);
    std::thread t2(ThreadSecond);
    t1.join();
    t2.join();

    return 0;
}