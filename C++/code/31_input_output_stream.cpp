/*
 * @Author: JohnJeep
 * @Date: 2020-07-14 15:09:12
 * @LastEditTime: 2020-07-15 10:42:43
 * @LastEditors: Please set LastEditors
 * @Description: 输入输出流
 * @FilePath: /31_input_output_stream.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <string.h>
#include <fstream>

using namespace std;

void test01()
{
    cout << "get函数测试用例" << endl;
    char t;
    cin >> t;

    while ((t = cin.get()) != EOF)  /* 键盘输入的数据到输入缓冲区中，一直阻塞到终端，
                                       直到遇见 EOF 文件结束符(ctrl + z)时，才退出输入 */
    {
        cout << t << endl;
    }
    cout << endl;
}

void test02()
{
    cout << "getline函数测试用例" << endl;
    char buf[16] = {0};
    char ch[16];
    cin >> ch;            // 终端输入的字符之间不能有空格

    cin.getline(buf, 16); // 解决终端输入的字符之间空格的问题
    cout << ch << endl;      
    cout << buf << endl;
    cout << endl;
}


// 输出流测试用例
void test03()
{
    cout << "输出流测试用例" << endl;
    cout.put('d').put('r').put('a').put('g').put('o').put('n') << endl;

    const char *str = "black"; 
    cout.write(str, strlen(str)) << endl;

    cout.width(20);                 // 标准输出数据的整个长度
    cout.fill('*');
    cout.setf(ios::showbase);    // 显示不同进制的前缀 0x
    cout.setf(ios::internal);    // 将输出数中的前缀和数据分开显示，中间插入填充的内容
    cout << hex << 123 << endl;
    cout << endl;
}

// 文件输入输出流测试用例
void test04()
{
    // 往文件中写数据
    cout << "文件输入输出流测试用例" << endl;
    const char *fname = "./test_file_iostream.txt";
    ofstream fout(fname);
    fout << "first line." << endl;
    fout << "second line." << endl;
    fout << "third line." << endl;
    fout.close();

    // 从文件中读数据，并将数据输出到终端上
    ifstream fin(fname);
    char ch;
    while (fin.get(ch))
    {
        cout << ch;
    }
    fin.close();
    cout << endl;
}



int main(int argc, char *argv[])
{
    // test01()
    test02();
    test03();
    test04();
    return 0;
}