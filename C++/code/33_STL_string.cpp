/*
 * @Author: JohnJeep
 * @Date: 2020-07-15 14:44:59
 * @LastEditTime: 2020-07-15 16:15:47
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库中的String
 * @FilePath: /33_STL_string.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <cstring>

using namespace std;

// 字符指针和string的转换：s.c_str()
// s.copy(buf, len, pos); 从 pos 位置拷贝 len 个长度的字符到 buf 中
// s.append()
// 字符串的查找s.find() 
// 字符串的替换s.replace(pos, len, str); 从 pos 位置开始，将源字符串中长度为 len 的字符替换为 str

void test01()
{
    string str = "hello string";
    const char*  pstr = str.c_str();   // 将string类的指针暴露出来
    cout << str << endl;
    cout << pstr << endl;    
    
    char buf[10] = {0};
    str.copy(buf, 5, 2);    // 返回结果为需要拷贝的长度 len
    cout << buf << endl;

    string s1 = "hello";
    string s2 = "C++";
    string s3 = s1 + s2;
    string s4 = s1.append(s2);   // 字符串连接
    cout << s3 << endl;
    cout << s4 << endl;
    cout << endl;
}

// 字符串的查找
void test02()
{
    string fstr = "this is a book, book is quite pretty, I love this book.";
    int index = fstr.find("book", 0);   // 返回当前找到字符的索引下标
    while (index != string::npos)
    {
        cout << "index索引下标：" << index << endl;
        index += 1;
        index = fstr.find("book", index);
    }
    
    cout << index << endl;
    cout << fstr << endl;
    cout << endl;
}

// 字符串的替换
void test03()
{
    string fstr = "this is a book, book is quite pretty, I love this book.";
    int index = fstr.find("book", 0);   // 返回当前找到字符的索引下标
    while (index != string::npos)
    {
        cout << "index索引下标：" << index << endl;
        fstr.replace(index, strlen("book"), "BOOK");
        index += 1;
        index = fstr.find("book", index);
    }
    
    cout << index << endl;
    cout << fstr << endl;
}
int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    return 0;
}