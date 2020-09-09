/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2020-09-09 23:34:36
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单利模式为懒汉式模式，即在new一个对象时，才分配内存
 *               在多个线程中，存在资源竞争的问题。
 *
 */
#include <iostream>
#include <stdlib.h>
#include <pthread.h>
#include <mutex>
#include <unistd.h>

using namespace std;

class Singleton
{
private:
    static pthread_mutex_t mutex;
    static Singleton* spl;
    Singleton();
    ~Singleton();
public:
    static int count;
    static Singleton* getInstance();        // 第一种方法
    static Singleton* getInstanceLock();    // 第二种方法
    static Singleton* getInstanceStatic();  // 第三种方法
    static Singleton* freeInstance();
    void show() {cout << "实例地址：" << this << endl;};
};

pthread_mutex_t mutex;
Singleton* Singleton::spl = nullptr; // 静态全局变量初始化

Singleton::Singleton()
{
    cout << "执行构造函数" << endl;
}

Singleton::~Singleton()
{
    cout << "执行析构函数" << endl;
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


// 第一种方法：懒汉式单例模式
Singleton* Singleton::getInstance()
{
    if (spl == nullptr)
    {
        spl = new Singleton;     // 只有在创建对象时，才分配内存
    }
    count++;

    return spl;
}

// 第二种方法添加同步锁后保证了多个线程同时访问时安全的
Singleton* Singleton::getInstanceLock()
{
    if (spl == nullptr)    // 双重检测机制
    {
        // unique_lock<std::mutex> m_mutex;
        pthread_mutex_lock(&mutex);  // 加同步锁，锁住整个类，防止new Singleton被执行多次
        // 进入临界区以后，两个线程同时访问时，需要保证一个时间内只有一个线程在创建对象
        if (spl == nullptr)
        {
            spl = new Singleton;     
        }
        pthread_mutex_unlock(&mutex);
    }
    return spl;
}

// 第三种方法：利用静态构造函数
Singleton* Singleton::getInstanceStatic()
{
    // 运行时确保只调用一次静态构造函数
    static Singleton *instance = new Singleton;
    return instance;
}

void *deal_thread(void * arg)
{
    pthread_t tid = pthread_self();
    pthread_t pid = getpid();
    cout << "tid = " << tid << "\t" << "pid = " << pid << endl;
    pthread_detach(tid);   // 主线程与子线程分离，两者相互不干涉，子线程结束同时子线程的资源自动回收
    

    Singleton::getInstance()->show();
    // cout << "count = " << tmp->count << endl;

    // Singleton *tmp = Singleton::getInstanceLock();
    // cout << "count = " << tmp->count << endl;

    // Singleton::getInstanceStatic()->show();

}

int main(int argc, char *argv[])
{
    cout << "执行懒汉式的单例模式" << endl;

    pthread_t tid;
    for (int i = 0; i < 5; i++)
    {
        // int ret = pthread_create(&tid, NULL, deal_thread, (void*)i);
        int ret = pthread_create(&tid, NULL, deal_thread, NULL);
        if (ret)
        {
            perror("pthread create error.");
        }
        sleep(2);
    }

    Singleton::freeInstance();

    return 0;
}



