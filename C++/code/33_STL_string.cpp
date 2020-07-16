/*
 * @Author: JohnJeep
 * @Date: 2020-07-15 14:44:59
 * @LastEditTime: 2020-07-16 10:47:53
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库中的String
 * @FilePath: /33_STL_string.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <iomanip>
#include <cctype>

using namespace std;

// 字符指针和string的转换：s.c_str()
// s.copy(buf, len, pos); 从 pos 位置拷贝 len 个长度的字符到 buf 中
// s.append()
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

// 字符串的查找 s.find() 
void test02()
{
    string fstr = "this is a book, book is quite pretty, I love this book.";
    int index = fstr.find("book", 0);   // 返回当前找到字符的索引下标
    while (index != -1)    // string::npos    -1
    {
        cout << "index索引下标：" << index << endl;
        index += 1;
        index = fstr.find("book", index);
    }
    
    cout << index << endl;
    cout << fstr << endl;
    cout << endl;
}

// 字符串的替换 s.replace(pos, len, str); 从 pos 位置开始，将源字符串中长度为 len 的字符替换为 str
void test03()
{
    string fstr = "this is a book, book is quite pretty, I love this book.";
    int index = fstr.find("book", 0);   // 返回当前找到字符的索引下标
    while (index != -1)
    {
        cout << "index索引下标：" << index << endl;
        fstr.replace(index, strlen("book"), "BOOK");
        index += 1;
        index = fstr.find("book", index);
    }
    
    cout << index << endl;
    cout << fstr << endl;
    cout << endl;
}

// 字符串的删除 s.erase(int pos, int n); 删除从 pos 位置开始的 n 个字符，返回修改后的字符串。
void test04()
{
    string estr = "big wold";
    estr.erase(1, 4);

    cout << "delete string: " << estr << endl;
    cout << endl;
}

// 字符串的插入 s.insert(int pos, const char *s); 在 pos 位置处插入字符串
// 字符串的插入 s.insert(int pos, int n, char c); 在 pos 位置处插入 n 个字符 C
void test05()
{
    string istr = "this is my pen";
    istr.insert(0, "Mike,");   // 头插法
    int index = istr.find("pen");
    istr.replace(index, 3, "PEN");
    istr.insert(istr.length(), " that my girl friend sent to me.");   // 尾插发

    cout << "insert string: " << istr << endl;
    cout << endl;
}

// 字符串的大小写转换 transform(); 
void test06()
{
    string tstr = "Cook is a good job.";
    transform(tstr.begin(), tstr.end(), tstr.begin(), ::toupper);
    cout << "tranfoem string: " << tstr << endl;
    cout << endl;
}

int main(int argc, char *argv[])
{
    // test01();
    // test02();
    // test03();
    // test04();
    test05();
    test06();
    
    return 0;
    
}