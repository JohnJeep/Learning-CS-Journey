/*
 * @Author: JohnJeep
 * @Date: 2020-07-27 22:33:14
 * @LastEditTime: 2020-07-27 22:54:55
 * @LastEditors: Please set LastEditors
 * @Description: 斐波拉数列: 输入一个数，求斐波那契数列的第 n 项
 * @FilePath: /07_Fibonacci.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

long long recursiveFibonacci(int n)
{
    if (n <= 0)
    {
        return 0;
    }
    if (n == 1)
    {
        return 1;
    }
    return recursiveFibonacci(n - 1) + recursiveFibonacci (n - 2);
}

long long loopFibonacci(int n)
{
    int result[2] = {0, 1};
    if (n < 2)
    {
        return result[n];
    }
    
    long long first = 1;
    long long second = 0;
    long long sum = 0;

    for (int i = 2; i <= n; i++)
    {
        sum = first + second;
        second = first;
        first = sum;
    }
    return sum;
}

void test01()
{
    int num = 2;
    long long result = recursiveFibonacci(num);
    cout << "recursive result: " << result << endl;

    long long val = loopFibonacci(num);
    cout << "loop val: " << val << endl;
}

int main(int argc, char *argv[])
{
    test01();
    return 0;
}