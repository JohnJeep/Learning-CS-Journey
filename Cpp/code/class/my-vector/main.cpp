/*
 * @Author: JohnJeep
 * @Date: 2020-07-12 09:05:46
 * @LastEditTime: 2021-05-20 22:13:03
 * @LastEditors: Please set LastEditors
 * @Description: 数组模板类的主体
 */ 
#include <iostream>
#include "MyVector.cpp"
using namespace std; 

// 测试用例1
void test01()
{
    MyVector<int> vec(10);
    for (int i = 0; i < vec.getLen(); i++)
    {
        vec[i] = i + 1;
        cout << vec[i] << " ";
    }
    cout << endl;

    MyVector<int> dvec = vec;
    for (int i = 0; i < dvec.getLen(); i++)
    {
        dvec[i] = i + 1;
        cout << dvec[i] << " ";
    }
    cout << endl;
}

// 测试用例1
void test02()
{
    MyVector<char> pve(4);   // 定义char类型的数组
    pve[0] = 'a';
    pve[1] = 'b';
    pve[2] = 'c';
    pve[3] = 'd';

    for (int i = 0; i < 4; i++)
    {
        cout << pve[i] << " ";
    }
    cout << endl;  

}

// 测试用例3
void test03()
{
    Teacher T1(30, "wang");
    Teacher T2(18, "li");
    Teacher T3(20, "zhou");

    MyVector<Teacher> tArray(3);  // 定义Teacher类型的数组
    tArray[0] = T1;
    tArray[1] = T2;
    tArray[2] = T3;
    for (int i = 0; i < 3; i++)
    {
        Teacher tmp = tArray[i];
        tmp.show();
    }
    
}

// 测试用例4: 需要对Teacher这个类进行优化

int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();

    return 0;
}
