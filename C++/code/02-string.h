/*
 * @Author: your name
 * @Date: 2020-05-28 14:08:52
 * @LastEditTime: 2020-05-28 15:29:45
 * @LastEditors: Please set LastEditors
 * @Description: String 头文件声明
*/ 
#ifndef __01_STRING_H
#define __01_STRING_H
#include <string.h>

class String
{
private:
    /* data */
    char* data;

public:
    String(const char* cstr = 0);          // 构造函数
    String(const String& str);             // 拷贝构造
    String& operator=(const String& str);  // 拷贝赋值
    ~String();
    // 设计一个函数
    char* get_c_str() const
    {
        return data;
    }
};

// 构造函数
inline String::String(const char* cstr)
{
    if(cstr)   //指针不为空 
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
inline String::~String()
{
    delete[] data;
}

// 拷贝构造函数
inline String::String(const String& str)
{
    data = new char[strlen(str.data) + 1];   // 注意：strlen(str.data)
    strcpy(data, str.data);
}


// 拷贝赋值函数：先删除，然后分配内存，最后拷贝
inline String& String::operator=(const String& str)
{
    if(this == &str) // 自拷贝
    {
        return *this;
    }

    delete[] data;
    data = new char[strlen(str.data)];
    strcpy(data, str.data);
    return *this;
} 
#endif
