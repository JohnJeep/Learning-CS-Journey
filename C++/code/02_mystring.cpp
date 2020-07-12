/*
 * @Author: JohnJeep
 * @Date: 2020-05-28 14:09:10
 * @LastEditTime: 2020-06-11 15:58:00
 * @LastEditors: Please set LastEditors
 * @Description: C++实现有指针的类
 */ 

#include <iostream>
#include "02-mystring.h"
using namespace std;

ostream& operator<<(ostream& os, MyString& sp)
{
    return os << sp.get_c_str();
}

int main()
{
    MyString s1("string1");
    
    MyString s2("Mystring2");
    cout << "s2 data: " << s2.get_c_str() << endl;

    MyString s3(s2);  // 调用拷贝构造函数
    cout << "s3 data: " << s3.get_c_str() << endl;

    // 将指针显示的露出来
    MyString *pstr = new MyString("pstr");  
    cout << "*pstr value: " << pstr->get_c_str() << endl;
    delete pstr;

    // 调用重载operator=
    MyString s4 = s2;   // 对象的初始化 等价于 MyString s4(s2);
    s4 = s1;
    cout << "s4 data: " << s4 << endl;
    
    // 调用operator[]
    MyString s5("string5");
    s5[1] = 'a';
    cout << "s5 data: " << s5.get_c_str() << endl;
    cout << "s5 data: " << s5 << endl;     // 调用重载operator<< 显示复杂的类型


    return 0;
}