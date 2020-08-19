/*
 * @Author: JohnJeep
 * @Date: 2020-08-18 09:13:43
 * @LastEditTime: 2020-08-18 09:47:17
 * @LastEditors: Please set LastEditors
 * @Description: 题目：二进制中 1 的个数
 *               描述：输入一个整数，输出该数32位二进制表示中1的个数。其中负数用补码表示。
 *               
 *               思路：如果一个整数不为0，那么这个整数至少有一位是1。如果我们把这个整数减1，那么原来处在整数最右边的1就会变为0，
 *                    原来在1后面的所有的0都会变成1(如果最右边的1后面还有0的话)。其余所有位将不会受到影响。
 */
#include <iostream>
#include <cstdio>

using namespace std;

int numberOfOne(int data)
{
    int count = 0;
    while (data != 0)
    {
        ++count;
        data = data & (data - 1);
    }
    
    return count;
}

int main(int argc, char *argv[])
{
    int ret = numberOfOne(12);
    cout << "ret = " << ret << endl;
    
    return 0;
}