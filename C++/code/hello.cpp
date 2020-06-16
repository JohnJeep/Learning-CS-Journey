/*
 * @Author: your name
 * @Date: 2020-05-27 08:55:46
 * @LastEditTime: 2020-06-16 14:39:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer\C++\code\hello.cpp
 */ 
#include <stdio.h>
#include <iostream>

template <typename T>
T add(T x, T y)
{
    return x + y;
}

int main()
{
    using namespace std;
    
    int a = 10;
    int b = 20;

    cout << add(a, b) << endl;
    cout << add(12.5, 10.2) << endl;
    
    return 0;
}