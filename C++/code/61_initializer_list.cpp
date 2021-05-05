/*
 * @Author: JohnJeep 
 * @Date: 2021-04-29 22:30:59
 * @LastEditTime: 2021-05-05 11:29:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <cstdlib>
#include <algorithm>

using namespace std;

int main(int argc, char *argv[])
{
    // int a = {3.14};    // error: 编译器不通过，使用 {} 之后，不能隐式的进行类型转换
    int b = 3.14;      // right: 编译通过，进行隐式的类型转

    cout << "Cpmare multi nums max: "<< max({100, 99, 34, 67, 868, 344}) << endl;

    cout << "Compare two nums max: " << max(100, 34) << endl;

    return 0;
}