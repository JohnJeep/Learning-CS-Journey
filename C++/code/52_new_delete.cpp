/*
 * @Author: JohnJeep
 * @Date: 2021-01-24 10:01:32
 * @LastEditTime: 2021-01-24 11:51:45
 * @LastEditors: Please set LastEditors
 * @Description: 测试环境：Win10 64 bit machine 
 *               深刻理解new、delete和new[]、delete[]的使用
 */

#include <iostream>
#include <stdlib.h>

using namespace std;

class Foo
{
public:
    int m_id;      // 4 byte
    long m_data;   // 4 byte
    string m_str;  // 32 byte

public:
    Foo();
    Foo(int i);

    virtual ~Foo();    // 添加virtual关键字后，Foo大小增加一个虚指针的大小
    static void* operator new(size_t size);
    static void  operator delete(void* pdead, size_t size);

    static void* operator new[](size_t size);
    static void  operator delete[](void* pdead, size_t size);

};

Foo::Foo(/* args */)
  :m_id(0)
{
    cout << "default ctor: this = " << this << ", id = " << m_id << endl;
}

Foo::Foo(int i)
  :m_id(i)
{
    cout << "overloading ctor: this = " << this << ", id = " << m_id << endl;
}

Foo::~Foo()
{
    cout << "default dtor: this = " << this << ", id = " << m_id << endl; 
}

void* Foo::operator new(size_t size)
{
    Foo* p = (Foo*)malloc(size);
    cout << "Foo::operator new: size = " << size <<  ", return: " << p << endl;
    return p;
}

void Foo::operator delete(void* pdead, size_t d_size)
{
    // d_size 参数由编译器指定，不要手动传入
    cout << "Foo::operator delete: pdead = " << pdead << ", size = " << d_size << endl;
    free(pdead);
}

void* Foo::operator new[](size_t size)
{
    Foo* p = (Foo*)malloc(size);
    cout << "Foo::operator new[]: size = " << size <<  ", return: " << p << endl;
    return p;    
}

void Foo::operator delete[](void* pdead, size_t d_size)
{
    // d_size 参数由编译器指定，不要手动传入
    cout << "Foo::operator delete[]: pdead = " << pdead << ", size = " << d_size << endl;
    free(pdead);
}

int main(int argc, char *argv[])
{
    cout << "sizeof(Foo) = " << sizeof(Foo) << endl;
    // testing new[] && delete[]
    Foo* f = new Foo(7);
    delete f;

    // testing new[] && delete[]
    Foo* arr = new Foo[5];  // size = 5*40 + 8
    delete[] arr;

    return 0;
}