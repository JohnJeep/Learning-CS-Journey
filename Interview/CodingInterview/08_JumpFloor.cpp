/*
 * @Author: JohnJeep
 * @Date: 2020-07-27 23:10:30
 * @LastEditTime: 2020-07-27 23:22:50
 * @LastEditors: Please set LastEditors
 * @Description: 青蛙跳台阶问题: 一只青蛙一次可以跳上1级台阶，也可以跳上2级。求该青蛙跳上一个n级的台阶总共有多少种跳法（先后次序不同算不同的结果）
 *               思路：把 n 阶台阶的跳法看成 n 的函数，记为：f(n)，当 n>=2时，每次跳一阶的跳法：f(n-1)
 *                     每次跳两阶的跳法为 f(n-2)，总的跳法为：f(n-1) + f(n-2)
 * @FilePath: /08_JumpFloor.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

int main(int argc, char *argv[])
{
    
    return 0;
}

