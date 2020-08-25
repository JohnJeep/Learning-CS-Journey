/*
 * @Author: JohnJeep
 * @Date: 2020-08-25 15:01:07
 * @LastEditTime: 2020-08-25 16:02:29
 * @LastEditors: Please set LastEditors
 * @Description: 题目：实现 strStr() 函数
 *               描述：给定一个 haystack 字符串和一个 needle 字符串，
 *                     在 haystack 字符串中找出 needle 字符串出现的第一个位置 (从0开始)。如果不存在，则返回-1。
 *                     示例： 输入: haystack = "hello", needle = "ll"
 *                            输出: 2
 *                           
 *                            输入: haystack = "aaaaa", needle = "bba"
 *                            输出: -1
 * 
 *               思路：利用两个索引，一个指向母字符串，一个指向子字符串。
 *                    1、子字符串为空时，返回结果为0，子字符串的长度达与母字符串的长度，返回 -1
 *                    2、遍历母字符串剩下的长度和子字符串相等的位置即可。
 *                    3、遍历每一个子字符串，与母字符串的字符一个一个的比较，
 *                       如果对应位置有不等的，则跳出循环，如果一直都没有跳出循环，则说明子字符串出现了，则返回起始位置。
 * 
 *               复杂度：(m-n)n
 */
#include <iostream>
#include <cstdio>
#include <string>

using namespace std;


int strStr(string haystack, string needle) 
{

    if (needle.empty())
    {
        return 0;
    }
    int m = haystack.size();
    int n = needle.size();
    if (n > m)
    {
        return -1;
    }
    for (int i = 0; i <= m - n; ++i)
    {
        int j = 0;
        for (j = 0; j < n; ++j)
        {
            if (haystack[i + j] != needle[j])
                break;
        }
        if (j == n)
        {
            return i;
        }
    }
    return -1;   // 没有子字符串
}

int main(int argc, char *argv[])
{
    
    return 0;
}