/*
 * @Author: JohnJeep
 * @Date: 2020-08-24 09:21:11
 * @LastEditTime: 2020-08-24 10:31:01
 * @LastEditors: Please set LastEditors
 * @Description: 题目：字符串的排列
 *               描述: 输入一个字符串,按字典序打印出该字符串中字符的所有排列。
 *                     例如输入字符串abc,则按字典序打印出由字符a,b,c所能排列出来的所有字符串abc,acb,bac,bca,cab和cba。
 *               思路：
 *                   1、将字符串分看成两部分，第一部分是它的第一个字符，第二部分是后面剩下的字符。
 *                   2、求所有可能出现在第一个位置的字符。把一个字符与后面的字符进行交换。
 *                   3、固定第一个字符，求后面字符的所有排列。可以把后面的字符也看成两部分：后面字符的第一个字符，
 *                      后面字符剩下的所有字符，将这两部分的字符逐次进行交换。不断递归的这样去划分，直到最后一个字符为 \0 时
 *                      退出递归，结束交换。
 */
#include <iostream>
#include <cstdio>
#include <vector>
#include <set>

using namespace std;

// 普通的递归方法实现
void permutation(char* str, char* begin)
{
    if (*begin == '\0')
    {
        printf("%s\n", str);
    }
    else
    {
        for (char* cur = begin; cur != '\0'; cur++)
        {
            char temp = *cur;
            *cur = *begin;
            *begin = temp;

            permutation(str, begin + 1);
            temp = *cur;
            *cur = *begin;
            *begin = temp;
        }
    }
}

void stringPermut(char* s)
{
    if (s == nullptr)
    {
        return;
    }
    permutation(s, s);
}

// 使用vector实现
void permutationCore(int pos, string s, set<string>& ret)
{
    if (pos + 1 == s.length())
    {
        ret.insert(s);
        return;
    }
    
    for (int i = 0; i < s.length(); i++)
    {
        swap(s[pos], s[i]);
        permutationCore(pos + 1, s, ret);
        swap(s[pos], s[i]);              // 回溯的原因：比如第二次交换后是"BAC"，需要回溯到"ABC"
                                         // 然后进行第三次交换，才能得到"CBA"
    }
}

vector<string> Permutation(string str) 
{
    if (str.empty())
    {
        return {};
    }
    set<string> ret;
    permutationCore(0, str, ret);

    return vector<string> (ret.begin(), ret.end());
}

int main(int argc, char *argv[])
{
    
    return 0;
}