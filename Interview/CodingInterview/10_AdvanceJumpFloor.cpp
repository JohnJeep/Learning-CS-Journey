/*
 * @Author: JohnJeep
 * @Date: 2020-07-27 23:19:19
 * @LastEditTime: 2020-07-28 22:13:53
 * @LastEditors: Please set LastEditors
 * @Description: 青蛙跳台阶加强：一只青蛙一次可以跳上1级台阶，也可以跳上2级……它也可以跳上n级。求该青蛙跳上一个n级的台阶总共有多少种跳法。
 *               思路：利用斐波那列数列的思想解决。
 *                     在n阶台阶，一次有1、2、...n阶的跳的方式时，总得跳法为：
 *                             | 1       ,(n=0 ) 
 *                     f(n) =  | 1       ,(n=1 )
 *                             | 2*f(n-1),(n>=2)
 * 
 * @FilePath: /10_AdvanceJumpFloor.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

int jumpFloorII(int number) 
{
    int val = 1;

    if (number == 1)
    {
        return val;
    }
    while ((number - 1) != 0)   // 也可以用 for 循环
    {
        --number;
        val *= 2;
    }
    return val;
}

int main(int argc, char *argv[])
{
    int result = jumpFloorII(5);
    cout << "result = " << result << endl;
    return 0;
}