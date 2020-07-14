/*
 * @Author: JohnJeep
 * @Date: 2020-07-14 15:09:12
 * @LastEditTime: 2020-07-14 16:13:44
 * @LastEditors: Please set LastEditors
 * @Description: 输入输出流
 * @FilePath: /31_input_output_stream.cpp
 */ 
#include <iostream>
#include <cstdio>

using namespace std;

void test01()
{
    char t;
    cin >> t;

    while ((t = cin.get()) != EOF)  /* 键盘输入的数据到输入缓冲区中，一直阻塞到终端，
                                       直到遇见 EOF 文件结束符(ctrl + z)时，才退出输入 */
    {
        cout << t << endl;
    }
}

void test02()
{
    char buf[16] = {0};
    char ch[16];
    cin >> ch;            // 终端输入的字符之间不能有空格

    cin.getline(buf, 16); // 解决终端输入的字符之间空格的问题
    cout << ch << endl;      
    cout << buf << endl;
}

int main(int argc, char *argv[])
{
    // test01()
    test02();
    
    return 0;
}