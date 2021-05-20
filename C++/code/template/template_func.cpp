/*
 * @Author: JohnJeep
 * @Date: 2020-06-15 09:06:47
 * @LastEditTime: 2021-05-20 22:26:53
 * @LastEditors: Please set LastEditors
 * @Description: 模板函数用例
 */ 
#include <iostream>
using namespace std;

// 采用引用的方法交换两个数
template <typename T>
void tswap(T &a, T &b)
{
    T c;
    c = a;
    a = b;
    b = c;
}

int main()
{
    int x = 11; 
    int y = 22; 
    tswap<int>(x, y);    // 显示调用
    cout << "x: " << x << endl;
    cout << "y: " << y << endl;

    char a = 'A'; 
    char b = 'B'; 
    tswap(a, b);       // 调用时自动类型推导
    cout << "a: " << a << endl;
    cout << "b: " << b << endl;

    return 0;
}