/*
 * @Author: JohnJeep
 * @Date: 2020-07-20 10:41:42
 * @LastEditTime: 2020-07-20 16:17:07
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /40_stl_algorithm.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
#include <map>

using namespace std;


void test01()
{

    // for_each();
    // find_if();
    // transform();
    int a[10]={9,6,3,8,5,2,7,4,1,0};
    for(int i=0;i<10;i++)
    cout<<a[i]<<endl;
   sort(a,a+10);
    for(int i=0;i<10;i++)
    cout<<a[i]<<endl;
}



int main(int argc, char *argv[])
{
    test01();
    return 0;
}