/*
 * @Author: JohnJeep
 * @Date: 2020-05-28 14:09:10
 * @LastEditTime: 2020-05-28 15:39:15
 * @LastEditors: Please set LastEditors
 * @Description: C++实现有指针的类
 */ 

#include <iostream>
#include "02-string.h"
using namespace std;

ostream& operator<<(ostream& os, String& sp)
{
    return os << sp;
}

int main()
{
    String s1();
    String s2("string2");
    cout << s2 << endl;

    String s3(s2);
    cout << s3 << endl;

    String *pstr = new String("pstr");
    delete pstr;
    cout << *pstr << endl;


    return 0;
}