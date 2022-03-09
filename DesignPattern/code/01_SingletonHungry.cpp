/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2022-03-07 22:16:36
 * @LastEditors: Please set LastEditors
 * @Description: 饿汉式单例模式，在类的全局区中一开始开辟空间为其创建对象
 *               在多个线程中，不存在资源竞争的问题。
 *             
 * 
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

class Singleton
{
public:
    static Singleton* getInstance();                 // 提供一个全局的静态方法
    static Singleton* freeInstance();                // 释放内存

private:
    static Singleton* m_Instance;                    // 静态指针 
    Singleton();                                     // 构造函数私有化，禁止他人创建
    ~Singleton();
};

Singleton* Singleton::m_Instance = new Singleton;    // 静态全局变量创建对象

Singleton::Singleton()
{
    cout << "执行构造函数" << endl;
}

Singleton::~Singleton()
{
    cout << "执行析构函数" << endl;
}

Singleton* Singleton::getInstance()
{
    return m_Instance;
}

Singleton* Singleton::freeInstance()
{
    if (m_Instance != nullptr) {
        delete m_Instance;
        m_Instance = nullptr;
        cout << "释放对象内存" << endl;
    }
    return m_Instance;
}

int main(int argc, char *argv[])
{
    cout << "执行饿汉式的单例模式" << endl;

    Singleton *s1 = Singleton::getInstance();
    Singleton *s2 = Singleton::getInstance();
    if (s1 == s2) {
        cout << "是同一个对象" << endl;
    }
    else {
        cout << "不是同一个对象" << endl;
    }
    
    Singleton::freeInstance();

    return 0;
}