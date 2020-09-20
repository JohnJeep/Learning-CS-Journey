/*
 * @Author: JohnJeep
 * @Date: 2020-07-09 19:03:55
 * @LastEditTime: 2020-09-20 18:34:51
 * @LastEditors: Please set LastEditors
 * @Description: 空格替换：把字符串中的每个空格替换为 %20
 */
#include <iostream>
#include <stdlib.h>
#include <string.h>

using namespace std;

void replaceBlank(char pstr[], int len)
{
    if (pstr == nullptr || len < 0)
    {
        return;
    }

    // 遍历字符串，得到空格数
    int i = 0;
    int oldLen = 0;
    int blankLen = 0;
    while (pstr[i] != '\0')
    {
        ++oldLen;
        if (pstr[i] == ' ')
        {
            ++blankLen;
        }
        ++i;
    }

    // 将空格替换为 %20
    int newLen = len + blankLen * 2;
    if (newLen < len)
    {
        return;
    }
    int indexNew = newLen; // 新字符串数组索引，索引下标从零开始
    int indexOld = oldLen; // 旧字符串数组索引
    while ((indexOld >= 0) && (indexNew > indexOld))
    {
        if (pstr[indexOld] == ' ')
        {
            pstr[indexNew--] = '0';
            pstr[indexNew--] = '2';
            pstr[indexNew--] = '%';
        }
        else
        {
            pstr[indexNew--] = pstr[indexOld]; // 将原来字符串从后向前依次复制到新的字符串中，同样是从后向前
        }
        --indexOld; // 每遍历一次，旧字符串数组索引减一
    }
}

// 测试用例
void test1()
{
    char str[] = "we are happy";
    // int len = sizeof(str);
    int len = strlen(str);

    cout << str << endl;
    replaceBlank(str, len);
    cout << str << endl;
}

// 空格位于字符串的中间、最后面、最前面、连续有多个空格
void test2()
{
    char str[] = " we are very  happy ";
    int len = strlen(str);

    cout << str << endl;
    replaceBlank(str, len);
    cout << str << endl;
}

// 字符串是一个nullptr指针、是个空格字符、是个空字符串
void test3()
{
    char str[] = " ";
    int len = strlen(str);

    cout << "空格字符: " << str << endl;
    replaceBlank(str, len);
    cout << "空格字符: " << str << endl;
}

void test4()
{
    char str[] = "";
    int len = 10;

    cout << "空字符串: " << str << endl;
    replaceBlank(str, len);
    cout << "空字符串: " << str << endl;
}

void test5()
{
    // char str[] = nullptr;
    int len = 0;

    cout << "字符串是nullptr指针: " << endl;
    replaceBlank(nullptr, len);
    cout << "字符串是nullptr指针: " << endl;
}

int main(int argc, char *argv[])
{
    test1();
    test2();
    test3();
    test4();
    test5();

    return 0;
}
