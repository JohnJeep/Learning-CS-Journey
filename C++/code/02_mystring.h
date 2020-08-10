/*
 * @Author: JohnJeep
 * @Date: 2020-05-28 14:08:52
 * @LastEditTime: 2020-08-08 23:21:24
 * @LastEditors: Please set LastEditors
 * @Description: MyString 头文件声明
*/ 
#ifndef __01_MYSTRING_H
#define __01_MYSTRING_H
#include <string.h>
#include <cstdio>

class MyString
{
private:
    char* data;

public:
    MyString();
    ~MyString();
    MyString(const char* cstr = 0);            // 有参构造函数
    MyString(const MyString& str);             // 拷贝构造
    MyString& operator=(const MyString& str);  // 操作运算符拷贝赋值
    char& operator[](const int index);
    char* get_c_str() const
    {
        return data;
    }
};

// 实现默认的无参构造函数
inline MyString::MyString()
{  
    printf("调用无参构造函数\n");
}


// 构造函数
inline MyString::MyString(const char* cstr)
{
    if(cstr != NULL)   //指针不为空 
    {
        data = new char[strlen(cstr)+1];
        strcpy(data, cstr);
    }
    else
    {
        data = new char[1];
        // strcpy(data, '\0');
        *data = '\0';
    }
}


// 析构函数
inline MyString::~MyString()
{
    delete[] data;
}

// 拷贝构造函数
inline MyString::MyString(const MyString& str)
{
    data = new char[strlen(str.data) + 1];   // 注意：strlen(str.data)
    strcpy(data, str.data);
}



/**
 * @description: 重载operator=运算符，进行拷贝赋值：先删除，然后分配内存，最后拷贝
 * @param :      若传入的参数不是引用而是实例，那么从形参到实参会调用一次拷贝构造函数。
 *               把参数声明为引用可以避免这样的无谓消耗，从而提高代码效率
 * @return:      对象的自身引用
 */
// 1、初级程序员的写法
#if 0
inline MyString& MyString::operator=(const MyString& str)
{
    if(this == &str) // 自拷贝，判断传入的参数和当前的实例是不是同一个实例
    {
        return *this;
    }

    delete[] data;
    data = nullptr;
    data = new char[strlen(str.data) + 1];
    strcpy(data, str.data);

    return *this;
} 
#endif 

/**
 * @description: 高级程序员的写法
 *               可能会出现的问题：在new一块新的内存是时，有可能内存不足，导致new char时，会抛出异常，违背了异常安全性的原则。
 *                解决方法：法一、先用new分配除一块新的内存（在构造函数中已经申请），再用delete释放已有的内存，
 *                               保证在当内存分配失败时，确保类的实例不会被修改。
 * 
 *                         法二、创建一个临时实例，再交换临时实例和原来的实例。
 * @param {type} 
 * @return {type} 
 */
#if 1
inline MyString& MyString::operator=(const MyString& str)
{
    if(this == &str) // 自拷贝，判断传入的参数和当前的实例是不是同一个实例
    {
        return *this;
    }

    MyString temp(str);
    char* ptr = temp.data;
    temp.data = data;
    data = ptr;   // 运行完后，自动调用析构函数把temp.data指向的内存释放调

    return *this;
} 
#endif

// 重载数组operator[]，返回数组的下标
char& MyString::operator[](const int index)
{
    return data[index];
}

// 两个复杂的对象进行对比
// 复杂的对象与常规的字符进行对比
// <, >, !=,

#endif
