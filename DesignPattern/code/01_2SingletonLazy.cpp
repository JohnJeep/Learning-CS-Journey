/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2022-03-21 00:29:52
 * @LastEditors: Please set LastEditors
 * @Description: 单例模式：此单例模式为懒汉式模式，即在 new 一个对象时，才分配内存
 *               在多个线程中，存在资源竞争的问题。
 *               
 *               可以参考：https://zhuanlan.zhihu.com/p/62014096
 */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

class Singleton
{
public:
    static Singleton* getInstance(); 
    static void freeInstance();

    void show() 
    {
        cout << "Instance address: " << this << "\n" << endl;
    }

private:
    Singleton();
    ~Singleton();
    Singleton(const Singleton &signal) = delete;
    const Singleton &operator=(const Singleton &signal) = delete;

    static Singleton* m_Instance;           // 静态指针    
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

Singleton* Singleton::getInstance()
{
    if (m_Instance == nullptr) {
        m_Instance = new (std::nothrow)Singleton;     // 只有在创建对象时，才分配内存
    }

    return m_Instance;
}

int main(int argc, char *argv[])
{
    cout << "Execute lazy singleton...\n" << endl;

    Singleton::getInstance()->show();
    Singleton::freeInstance();

   return 0;
}