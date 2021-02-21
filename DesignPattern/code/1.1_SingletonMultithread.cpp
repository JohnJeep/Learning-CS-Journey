/*
 * @Author: JohnJeep
 * @Date: 2021-02-21 11:07:51
 * @LastEditTime: 2021-02-21 11:23:11
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式在多线程中，线程安全的实现。
 *               使用的<mutex>类需要C++11之后才可以，Mingw编译器下需要高版本才支持，
 *               低版本需要添加额外的环境，Linux环境下没有此问题。
 */
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>

using namespace std;

class Singleton
{
    /**
     * The Singleton's constructor/destructor should always be private to
     * prevent direct construction/desctruction calls with the `new`/`delete`
     * operator.
     */
private:
    Singleton(const std::string value)
        : m_value(value)
    {
    }
    ~Singleton()
    {
    }

    std::string m_value;
    static Singleton *m_pinstance;
    static std::mutex m_mutex;

public:
    /**
     * Singletons should not be cloneable.
     */
    Singleton(Singleton &other) = delete;

    /**
     * Singletons should not be assignable.
     */
    void operator=(const Singleton &) = delete;

    /**
     * This is the static method that controls the access to the singleton
     * instance. On the first run, it creates a singleton object and places it
     * into the static field. On subsequent runs, it returns the client existing
     * object stored in the static field.
     */

    static Singleton *GetInstance(const std::string &value);
    /**
     * Finally, any singleton should define some business logic, which can be
     * executed on its instance.
     */

    std::string value() const
    {
        return m_value;
    }
};

/**
 * Static methods should be defined outside the class.
 */
Singleton *Singleton::m_pinstance{nullptr};
std::mutex Singleton::m_mutex;

/**
 * The first time we call GetInstance we will lock the storage location
 *      and then we make sure again that the variable is null and then we
 *      set the value. 
 */
Singleton *Singleton::GetInstance(const std::string &value)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_pinstance == nullptr)
    {
        m_pinstance = new Singleton(value);
    }
    return m_pinstance;
}

void ThreadFoo()
{
    // Following code emulates slow initialization.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Singleton *singleton = Singleton::GetInstance("FOO");
    std::cout << singleton->value() << "\n";
}

void ThreadBar()
{
    // Following code emulates slow initialization.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    Singleton *singleton = Singleton::GetInstance("BAR");
    std::cout << singleton->value() << "\n";
}

int main(int argc, char *argv[])
{
    std::cout << "Singleton multithread safe" << endl;
    std::thread t1(ThreadFoo);
    std::thread t2(ThreadBar);
    t1.join();
    t2.join();

    return 0;
}