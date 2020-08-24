/*
 * @Author: JohnJeep
 * @Date: 2020-07-27 23:10:30
 * @LastEditTime: 2020-07-28 22:13:33
 * @LastEditors: Please set LastEditors
 * @Description: 青蛙跳台阶问题: 一只青蛙一次可以跳上1级台阶，也可以跳上2级。求该青蛙跳上一个 n 级的台阶总共有多少种跳法（先后次序不同算不同的结果）
 * 
 *    思路：  对于本题,前提只有 一次 1阶或者2阶的跳法。
 * a. 如果两种跳法，1阶或者2阶，那么假定第一次跳的是一阶，那么剩下的是n-1个台阶，跳法是f(n-1);
 * b. 假定第一次跳的是2阶，那么剩下的是n-2个台阶，跳法是f(n-2)
 * c. 由a和b假设可以得出总跳法为: f(n) = f(n-1) + f(n-2) 
 * d. 然后通过实际的情况可以得出：只有一阶的时候 f(1) = 1 ,只有两阶的时候可以有 f(2) = 2
 * e. 可以发现最终得出的是一个斐波那契数列：       
 *          | 1, (n=1)
 * f(n) =   | 2, (n=2)
 *          | f(n-1)+f(n-2) ,(n>2,n为整数)
 * 
 * @FilePath: /10_JumpFloor.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

// 第一种写法
int jumpFloor1(int number)
{
    int ret[2] = {0, 1};
    if (number < 2)
    {
        return ret[number];
    }
    int t1 = 0;
    int t2 = 1;
    int sum = 0;
    for (int i = 0; i < number; i++)    // 关键点：i 从零开始，i < number
    {
        sum = t2 + t1;
        t1 = t2;
        t2 = sum;
    }
    return sum;
}

// 第二种写法
int jumpFloor2(int number)
{
    if (number == 1 || number == 2)
    {
        return number;
    }

    int n1 = 1;
    int n2 = 2;
    int sum = 0;
    for (int i = 2; i < number; i++)    
    {
        sum = n2 + n1;
        n1 = n2;
        n2 = sum;
    }
    return sum;
}

int main(int argc, char *argv[])
{
    int val = jumpFloor1(4);
    int res = jumpFloor2(4);
    cout << val << endl;
    cout << res << endl;
    
    return 0;
}

