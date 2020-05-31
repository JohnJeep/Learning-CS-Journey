/*
 * @Author: JohnJeep
 * @Date: 2020-05-31 10:55:48
 * @LastEditTime: 2020-05-31 12:38:22
 * @LastEditors: Please set LastEditors
 * @Description: 引用的简单例子
 */ 
#include <iostream>
#include <stdio.h>
using namespace std;


typedef struct reference
{
    char name[10];
    int age;
    int const &a = 0;
    int const &b = 0;
}ref;

void test01(int& a)
{
    a = 10;
}

void test02(int*  a)
{
    *a = 20;
}

int leftValue()
{
    static int x = 30;
    cout << "x value:" << x << endl;
    
    return x;
}

int& leftValueRef()
{
    static int y = 30;
    cout << "y value:" << y << endl;

    return y;
}


int main()
{
    ref ref_t;
    
    printf("C printf: %d\n", sizeof(ref_t));
    cout << sizeof(ref_t) << endl;

    int s = 11;  
    test01(s);
    cout << "s: " << s << endl;

    int tmp = 100;
    int* t = &tmp;
    cout << "tmp: " << tmp << endl;
    test02(t); 
    cout << "t: " << *t << endl;
    cout << endl;


    int ret = 0;
    // leftValue() = 100;
    leftValueRef() = 100;  // leftValueRef() 等价于 y
    cout << "leftValueRef() value:" << leftValueRef() << endl;
    cout << endl;

    // 作为右值时
    ret = leftValueRef();  
    cout << "leftValueRef() value:" << ret << endl;
    cout << endl;

    int val = 0;
    val = leftValue();  
    cout << "leftValue() value:" << val << endl;
    // cout << endl;


    return 0;
}








