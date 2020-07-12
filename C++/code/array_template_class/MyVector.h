/*
 * @Author: JohnJeep
 * @Date: 2020-07-12 09:05:34
 * @LastEditTime: 2020-07-12 18:02:51
 * @LastEditors: Please set LastEditors
 * @Description: 数组模板类的声明
 * @FilePath: /MyVector.h
 */ 
#include <iostream>
using namespace std; 

template <typename T>
class MyVector
{
private:
    int m_len;
    T *m_array;
    
public:
    MyVector(int len);
    MyVector(const MyVector& obj);

    ~MyVector();

    int getLen() {return m_len;}
    T& operator[] (int index);
    MyVector& operator= (const MyVector& obj);
    
    template <typename u>
    friend ostream& operator<< (ostream& out, const MyVector<u>& obj);
};


class Teacher
{
private:
    int m_age;
    char m_name[16];
    // char *m_name;
public:
    Teacher();
    Teacher(int age, const char *name);
    ~Teacher();
    void show();
    
};

