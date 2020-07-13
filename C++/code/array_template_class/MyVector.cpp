/*
 * @Author: JohnJeep
 * @Date: 2020-07-11 09:50:29
 * @LastEditTime: 2020-07-13 14:06:37
 * @LastEditors: Please set LastEditors
 * @Description: 数组模板类的实现
 * @FilePath: /MyVector.cpp
 */ 
#include <iostream>
#include "MyVector.h"
#include "string.h"

using namespace std; 

// 构造函数
template <typename T>
MyVector<T>::MyVector(int len)
{
    m_array = new T[len];
    m_len = len;
}

// 拷贝构造函数
template <typename T>
MyVector<T>::MyVector(const MyVector& obj)
{
    // 根据原数组的大小给新数组分配空间
    m_len = obj.m_len;
    m_array = new T[m_len];

    // copy数据
    for (int i = 0; i < m_len; i++)
    {
        m_array[i] = obj.m_array[i];  // 原数组值copy给新数组
    }
}

// 析构函数
template <typename T>
MyVector<T>::~MyVector()
{
    if (m_array != NULL)
    {
        delete []m_array;
        m_array = NULL;
        m_len = 0;
    }
    
}

// 重载[]运算符
template <typename T>
T& MyVector<T>::operator[] (int index)
{
    return m_array[index];
}

// 重载=运算符
template <typename T>
MyVector<T>& MyVector<T>::operator= (const MyVector<T>& obj)
{
    // 先释放需要拷贝的目标数组中旧的内存
    if (m_array != NULL)
    {
        delete []m_array; 
        m_array = nullptr;
        m_len = 0;
    }

    // 新数组分配空间
    m_len = obj.m_len;
    m_array = new T[m_len];

    // copy数据
    for (int i = 0; i < m_len; i++)
    {
        m_array[i] = obj.m_array[i];
    }
    return *this;
}

// 重载<<运算符
template <typename u>
ostream& operator<< (ostream& out, const MyVector<u>& obj)
{
    for (int i = 0; i < obj.m_len; i++)
    {
        cout << obj.m_array[i] << " ";
    }
    cout << endl;
    
    return out;
}


// teacher类的实现
Teacher::Teacher() // 默认构造函数
{
    strcpy(m_name, " ");
}

Teacher::Teacher(int age, const char *name)  // 构造函数重载
{
    this->m_age = age;
    strcpy(this->m_name, name);
}

Teacher::~Teacher()
{
}

void Teacher::show()
{
    cout << m_name << "," << m_age << endl;
}
