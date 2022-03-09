/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2022-03-08 01:10:47
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单例模式为懒汉式模式，即在 new 一个对象时，才分配内存
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
public:
    static Singleton* getInstance();        // 第一种方法
    // static Singleton* getInstanceLock();    // 第二种方法
    // static Singleton* getInstanceStatic();  // 第三种方法
    static void freeInstance();

    void show() 
    {
        cout << "Instance address: " << this << "\n" << endl;
    }

private:
    Singleton();
    ~Singleton();
    static Singleton* m_Instance;           // 静态指针    

    Singleton(const Singleton &signal) = delete;
    const Singleton &operator=(const Singleton &signal) = delete;
};

// 类外部静态全局变量初始化
Singleton* Singleton::m_Instance = nullptr; 

Singleton::Singleton()
{
    cout << "Execute constructor." << endl;
}

Singleton::~Singleton()
{
    cout << "Execute destructor." << endl;
}

void Singleton::freeInstance()
{
    if (m_Instance != nullptr) {
        delete m_Instance;
        m_Instance = nullptr;
        cout << "Free instance memory." << endl;
    }
}

// 第一种方法：懒汉式单例模式
Singleton* Singleton::getInstance()
{
    if (m_Instance == nullptr) {
        m_Instance = new (std::nothrow)Singleton;     // 只有在创建对象时，才分配内存
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
// Singleton* Singleton::getInstanceLock()
// {
//     if (m_Instance == nullptr) {
//         // 加同步锁，锁住整个类，防止 new Singleton 被执行多次
//         m_mutex.lock();
//         if (m_Instance == nullptr) {
//             m_Instance = new Singleton;     
//         }
//         m_mutex.unlock();
//     }
//     return m_Instance;
// }

// 第三种方法：利用静态构造函数
// Singleton* Singleton::getInstanceStatic()
// {
//     // 运行时确保只调用一次静态构造函数
//     static Singleton *instance = new Singleton;
//     return instance;
// }

// 线程处理函数
void *deal_thread(void * arg)
{
    pthread_t tid = pthread_self();          // 获得调用线程的线程ID号
    // pthread_t pid = getpid();
    // cout << "tid = " << tid << "\t" << "pid = " << pid << endl;
    pthread_detach(tid);                     // 主线程与子线程分离，两者相互不干涉，
                                            //  子线程结束同时子线程的资源自动回收
    
    // 调用第一种方法
    Singleton::getInstance()->show();
    
    // 调用第二种方法
    // Singleton::getInstanceLock()->show();

    // 调用第三种方法
    // Singleton::getInstanceStatic()->show();
    pthread_exit(NULL);
}

void *PrintHello(void *threadid)
{
    // 主线程与子线程分离，两者相互不干涉，子线程结束同时子线程的资源自动回收
    pthread_detach(pthread_self());

    // 对传入的参数进行强制类型转换，由无类型指针变为整形数指针，然后再读取
    int tid = *((int *)threadid);

    std::cout << "Hi, 我是线程 ID:[" << tid << "]" << std::endl;

    // 打印实例地址
    Singleton::getInstance()->show();

    pthread_exit(NULL);
}

#define NUM_THREADS 5

int main(int argc, char *argv[])
{
    cout << "Execute lazy singleton...\n" << endl;

    // pthread_t tid[5] = {0};
    // int index[5] = {0};
    // int i = 0;
    // int ret =0;

    // // 创建线程
    // for (i = 0; i < 5; i++) {
    //     index[i] = i;
    //     int ret = pthread_create(&tid[i], NULL, deal_thread, (void*)&(index[i]));
    //     if (ret) {
    //         perror("pthread create error.\n");
    //         exit(-1);
    //     }
    //     // sleep(1);
    // }

    pthread_t threads[NUM_THREADS] = {0};
    int indexes[NUM_THREADS] = {0}; // 用数组来保存i的值

    int ret = 0;
    int i = 0;

    for (i = 0; i < NUM_THREADS; i++)
    {
		indexes[i] = i; //先保存i的值
		
        // 传入的时候必须强制转换为void* 类型，即无类型指针
        ret = pthread_create(&threads[i], NULL, PrintHello, (void *)&(indexes[i]));
        if (ret)
        {
            std::cout << "Error:无法创建线程," << ret << std::endl;
            exit(-1);
        }
    }
    Singleton::freeInstance();

   return 0;
}