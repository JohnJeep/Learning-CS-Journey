/*
 * @Author: JohnJeep
 * @Date: 2021-02-20 17:14:47
 * @LastEditTime: 2022-03-07 23:54:50
 * @LastEditors: Please set LastEditors
 * @Description: 模拟懒汉模式资源竞争
 */
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

using namespace std;

class Singleton
{
public:
    static Singleton* getInstace(const std::string& value); // 第一种方法
    static Singleton* getInstanceLock();                    // 第二种方法
    static Singleton* getInstanceStatic();                  // 第三种方法
    static Singleton* freeInstance();

    // 禁止拷贝操作
    Singleton(Singleton& other) = delete;

    // 禁止赋值操作
    void operator=(const Singleton&) = delete;
    
    std::string getValue() {
        return m_value;
    }

private:
    Singleton(const std::string value);     // constructor
    ~Singleton();                           // destructor
    static Singleton* m_Instance;           // 静态成员指针
    std::string m_value;
};

// 静态变量外部初始化
Singleton* Singleton::m_Instance = nullptr;

Singleton::Singleton(const std::string value)
    : m_value(value)
{
}

Singleton::~Singleton()
{
}

// 第一种：没有加锁，存在多个线程访问时资源竞争的问题
Singleton* Singleton::getInstace(const std::string& value)
{
    if (m_Instance == nullptr) {
        m_Instance = new Singleton(value);
    }
    return m_Instance;
}

/**
 * @description: 第二种方法添加同步锁后保证了多个线程同时访问时安全的
 *               两次变量判断。双重检测机制，进入临界区以后，两个线程同时访问时，
 *               需要保证一个时间内只有一个线程在创建对象
 * @param {type} 
 * @return {type}: m_Instance
 */
Singleton* Singleton::getInstanceLock()
{
    if (m_Instance == nullptr) {
        // 加同步锁，锁住整个类，防止 new Singleton 被执行多次
        m_mutex.lock();
        if (m_Instance == nullptr) {
            m_Instance = new Singleton;     
        }
        m_mutex.unlock();
    }
    return m_Instance;
}

// 第三种方法：利用静态构造函数
Singleton* Singleton::getInstanceStatic()
{
    // 运行时确保只调用一次静态构造函数
    static Singleton *instance = new Singleton;
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
    std::cout << singleton->getValue() << std::endl;
}

// 第二个线程处理函数
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
    t1.join();                          // 回收创建的线程，避免资源浪费
    t2.join();

    return 0;
}