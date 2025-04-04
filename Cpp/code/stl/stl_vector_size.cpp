/*
 * @Author: JohnJeep
 * @Date: 2020-07-26 11:17:53
 * @LastEditTime: 2021-05-21 16:15:31
 * @LastEditors: Please set LastEditors
 * @Description: size()与capacity()STL库函数的区别
 */ 
#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace std;

void show(vector<int>& v)
{
    for (int i = 0; i < 10; i++) {
        v[i] = i + 1;
    }    
    for (vector<int>::iterator it = v.begin(); it != v.end(); it++) {
        cout << *it << " ";
    }
    cout << endl;    
    cout << "size = " << v.size() << "\t" << "capicity = " << v.capacity() << endl;
    cout << endl;
}

int main(int argc, char *argv[])
{

    vector<int> vt;

    cout << "size = " << vt.size() << "\t" << "capicity = " << vt.capacity() << endl;

    // 容器中预留的内存空间，并没有真正分配内存空间，即此时空间是野的。
    // 此时使用[]操作符访问容器内的对象，很可能出现数组越界的问题。
    vt.reserve(10);   
    cout << "size = " << vt.size() << "\t" << "capicity = " << vt.capacity() << endl;

    cout << "execute reserve: ";
    show(vt);
    
    cout << "execute resize: ";
    vt.resize(10);
    show(vt);

    cout << "execute push_back: ";
    vt.push_back(100);
    vt.push_back(200);
    show(vt);
    
    return 0;
}