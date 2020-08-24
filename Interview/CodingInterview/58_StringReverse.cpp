/*
 * @Author: JohnJeep
 * @Date: 2020-08-24 10:43:14
 * @LastEditTime: 2020-08-24 16:17:44
 * @LastEditors: Please set LastEditors
 * @Description: 题目：翻转字符串
 *               描述：翻转单词顺序
 *               描述：输入一个英文单词，翻转句子中单词的顺序，但单词内字符的顺序不变。
 *                     例如：输入字符串"I am a student."，则输出"student. a am I"
 *               思路：
 *                   1、翻转整个句子的顺序
 *                   2、翻转每个单词的顺序
 * 
 */
#include <iostream>
#include <cstdio>
#include <string>

using namespace std;

void reverse(char* begin, char* end)
{
    if (begin == nullptr || end == nullptr)
    {
        return;
    }
    while (begin < end)
    {
        char temp = *begin;
        *begin = *end;
        *end = temp;
        begin++;
        end--;   // 每执行一次end指针减一，end指针开始的位置时，由传入参数决定
    }
}

char* reverseSentence(char* str)
{
    if (str == nullptr)
    {
        return nullptr;
    }
    
    char* begin = str;
    char* end = str;
    while (*end != '\0')
    {
        end++;        // 将指针指向句子的结尾
    }
    end--;

    // 翻转这个句子
    reverse(begin, end);

    // 翻转句子中的单词
    begin = end = str;    // 重新将指针指向字符串的开始位置
    while (*begin != '\0')
    {
        if (*begin == ' ')
        {
            begin++;
            end++;
        }
        else if (*end == ' ' || *end == '\0')
        {
            reverse(begin, --end); // 传入的end指向的值不能为空或\0
            begin = ++end;         // 将开始begin指针指向每个单词的最后一个位置；++优先级高于=
        }
        else
        {
            end++;
        }        
    }
    return str;
}

// 使用string类实现
void reverseString(string&str, int begin, int end)
{
    while (begin < end)
    {
        char temp = str[begin];
        str[begin] = str[end];
        str[end] = temp;
        begin++;
        end--;   // 每执行一次end指针减一，end指针开始的位置时，由传入参数决定
    }
}
string ReverseSentenceString(string str) 
{
    if (str.empty())
    {
        return nullptr;
    }
    
    int begin = 0;
    int end = 0;

    // 翻转句子中的单词
    while (end <= str.length())
    {
        if (str[end] == ' ' || end == str.size())
        {
            reverseString(str, begin, end - 1);
            begin = end + 1;
        }
        end++;    
    }

    // 翻转这个句子
    reverseString(str, 0, str.size() - 1);    
    return str;   
}

int main(int argc, char *argv[])
{
    char pstr[] = "I am a student.";
    char* result = reverseSentence(pstr);
    printf("%s\n", result);

    string p = "I am a student.";
    string ret = ReverseSentenceString(p);
    cout << ret << endl;

    return 0;
}