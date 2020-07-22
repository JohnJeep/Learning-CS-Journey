/*
 * @Author: your name
 * @Date: 2020-07-22 15:51:02
 * @LastEditTime: 2020-07-22 16:00:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Journey\C++\code\test.cpp
 */ 
#include <iostream>
#include <cstdio>

using namespace std;

class test
{
private:
    int m_data[4];
    string name;
public:
    test(/* args */);
    ~test();
    int getData(int n) const {return m_data[n];}
    void setData(int data, int i) {this->m_data[i] = data;}
};

test::test(/* args */)
{
}

test::~test()
{
}


int main(int argc, char *argv[])
{
    test t1;
    t1.setData(12, 1);
    cout << t1.getData(1) << endl;
    
    return 0;
}