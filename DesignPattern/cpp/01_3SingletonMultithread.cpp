/*
 * @Author: JohnJeep
 * @Date: 2021-02-20 17:14:47
 * @LastEditTime: 2022-03-22 01:07:27
 * @LastEditors: Please set LastEditors
 * @Description: 模拟懒汉模式资源竞争：线程安全与不安全问题
 */
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>

using namespace std;

class Singleton
{
public:
    static Singleton* getInstace(const std::string& value);               // 懒汉式
    static Singleton* getInstanceLock(const std::string& value);          // 加双重锁
    static Singleton* getInstanceStatic(const std::string& value);        // 静态构造
    static Singleton* freeInstance();

    Singleton(Singleton& other) = delete;                                 // 禁止拷贝操作
    const Singleton& operator=(const Singleton&) = delete;                // 禁止赋值操作
    
    std::string getValue() {
        return m_value;
    }

private:
    // GC 类：程序结束时，进入析构函数销毁 Singleton 类的实例
    class GC
    {
    public:
        GC() {}
        ~GC() {
            if (m_Instance != nullptr) {
                delete m_Instance;
                m_Instance = nullptr;
                cout << "Exec GC dector, delete m_Instance.\n";
            }
        }
    };

    static GC gc;                           // 定义静态成员变量，当程序结束时，会调用 GC 类的析构函数

private:
    Singleton(const std::string value);     // constructor
    ~Singleton();                           // destructor

    static Singleton* m_Instance;           // 静态成员指针
    static std::mutex m_mutex;
    std::string m_value;
};

// 静态变量外部初始化
Singleton::GC Singleton::gc; 
Singleton* Singleton::m_Instance = nullptr;
std::mutex Singleton::m_mutex;

Singleton::Singleton(const std::string value)
    : m_value(value)
{
    cout << "Exec Singleton ctor.\n";
}

Singleton::~Singleton()
{
    cout << "Exec Singleton dector.\n";
}

// 懒汉式单例，存在多个线程访问时资源竞争的问题
Singleton* Singleton::getInstace(const std::string& value)
{
    if (m_Instance == nullptr) {
        m_Instance = new Singleton(value);
    }
    return m_Instance;
}

/**
 * @description: 添加同步锁后保证了多个线程同时访问时安全的
 *               两次变量判断。双重检测机制，进入临界区以后，两个线程同时访问时，
 *               需要保证一个时间内只有一个线程在创建对象
 * @param {type} 
 * @return {type}: m_Instance
 */
Singleton* Singleton::getInstanceLock(const std::string& value)
{
    if (m_Instance == nullptr) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_Instance == nullptr) {
            m_Instance = new Singleton(value);     
        }
    }
    return m_Instance;
}

// 利用静态构造函数，线程安全
Singleton* Singleton::getInstanceStatic(const std::string& value)
{
    // 运行时确保只调用一次静态构造函数
    static Singleton *instance = new Singleton(value);
    return instance;
}

Singleton* Singleton::freeInstance()
{
    if (m_Instance != nullptr) {
        delete m_Instance;
        m_Instance = nullptr;
        cout << "Free instance memory." << endl;
    }
    return m_Instance;
}

// 第一个线程处理函数
void ThreadFirst()
{
    std::this_thread::sleep_for(chrono::milliseconds(1000));
    Singleton* singleton = Singleton::getInstace("First");
    // Singleton* singleton = Singleton::getInstanceLock("First");
    // Singleton* singleton = Singleton::getInstanceStatic("First");
    std::cout << singleton->getValue() << std::endl;
}

// 第二个线程处理函数
void ThreadSecond()
{
    std::this_thread::sleep_for(chrono::milliseconds(1000));
    Singleton* singleton = Singleton::getInstace("Second");
    // Singleton* singleton = Singleton::getInstanceLock("Second");
    // Singleton* singleton = Singleton::getInstanceStatic("Second");
    std::cout << singleton->getValue() << std::endl;
}

int main(int argc, char *argv[]) 
{
    std::thread t1(ThreadFirst);
    std::thread t2(ThreadSecond);
    t1.join();                          // 回收创建的线程，避免资源浪费
    t2.join();

    return 0;
}