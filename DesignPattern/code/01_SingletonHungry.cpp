/*
 * @Author: JohnJeep
 * @Date: 2020-08-06 22:19:11
 * @LastEditTime: 2020-09-08 16:01:04
 * @LastEditors: Please set LastEditors
 * @Description: 饿汉式单利模式，在类的全局区中一开始开辟空间为其创建对象
 *               在多个线程中，不存在资源竞争的问题。
 *             
 * 
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

class Singleton
{
private:
    static Singleton* spl;
    Singleton();
public:
    ~Singleton();
    static Singleton* getInstance();
    static Singleton* freeInstance();

};

Singleton* Singleton::spl = new Singleton;     // 静态全局变量创建对象

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
    return spl;
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


int main(int argc, char *argv[])
{
    cout << "执行饿汉式的单例模式" << endl;

    Singleton *s1 = Singleton::getInstance();
    Singleton *s2 = Singleton::getInstance();
    if (s1 == s2)
    {
        cout << "是同一个对象" << endl;
    }
    else
    {
        cout << "不是同一个对象" << endl;
    }
    
    Singleton::freeInstance();

    return 0;
}



