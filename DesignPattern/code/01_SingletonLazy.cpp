/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2021-02-21 17:16:36
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单利模式为懒汉式模式，即在new一个对象时，才分配内存
 *               在多个线程中，存在资源竞争的问题。
 *               
 *               可以参考：https://zhuanlan.zhihu.com/p/62014096
 *               注意：Windows下MinGW编译器编译可能需要配置<Mutex>的环境
 *
 */
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <mutex>
#include <unistd.h>

using namespace std;
std::mutex m_mutex;

class Singleton
{
private:
    Singleton();
    ~Singleton();
    static Singleton* m_singleton;

public:
    static Singleton* getInstance();        // 第一种方法
    static Singleton* getInstanceLock();    // 第二种方法
    static Singleton* getInstanceStatic();  // 第三种方法
    static Singleton* freeInstance();

    void show() 
    {
        cout << "Instance address: " << this << endl;
    }
};

// 静态全局变量初始化
Singleton* Singleton::m_singleton = nullptr; 

Singleton::Singleton()
{
    cout << "Execute constructor." << endl;
}

Singleton::~Singleton()
{
    cout << "Execute destructor." << endl;
}

Singleton* Singleton::freeInstance()
{
    if (m_singleton != nullptr) {
        delete m_singleton;
        m_singleton = nullptr;
    }
    cout << "Free instance memory." << endl;
    return m_singleton;
}

// 第一种方法：懒汉式单例模式
Singleton* Singleton::getInstance()
{
    if (m_singleton == nullptr) {
        m_singleton = new Singleton;     // 只有在创建对象时，才分配内存
    }

    return m_singleton;
}

/**
 * @description: 第二种方法添加同步锁后保证了多个线程同时访问时安全的
 *               两次变量判断。双重检测机制，进入临界区以后，两个线程同时访问时，
 *               需要保证一个时间内只有一个线程在创建对象
 * @param {type} 
 * @return {type}: m_singleton
 */
Singleton* Singleton::getInstanceLock()
{
    if (m_singleton == nullptr)    
    {
        // 加同步锁，锁住整个类，防止new Singleton被执行多次
        m_mutex.lock();
        if (m_singleton == nullptr)                                           
        {
            m_singleton = new Singleton;     
        }
        m_mutex.unlock();
    }
    return m_singleton;
}

// 第三种方法：利用静态构造函数
Singleton* Singleton::getInstanceStatic()
{
    // 运行时确保只调用一次静态构造函数
    static Singleton *instance = new Singleton;
    return instance;
}

// 线程处理函数
void *deal_thread(void * arg)
{
    pthread_t tid = pthread_self();
    pthread_t pid = getpid();
    cout << "tid = " << tid << "\t" << "pid = " << pid << endl;
    pthread_detach(tid);   // 主线程与子线程分离，两者相互不干涉，子线程结束同时子线程的资源自动回收
    
    Singleton* singleton = Singleton::getInstance();
    singleton->show();
    
    Singleton *tmp = Singleton::getInstanceLock();
    Singleton::getInstanceStatic()->show();
}

int main(int argc, char *argv[])
{
    cout << "Execute lazy singleton..." << endl;

    pthread_t tid;

    for (int i = 0; i < 5; i++) {
        int ret = pthread_create(&tid, NULL, deal_thread, (void*)i);
        if (ret) {
            perror("pthread create error.");
        }
        sleep(2);
    }

    Singleton::freeInstance();

    return 0;
}

