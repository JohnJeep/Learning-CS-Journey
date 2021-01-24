/*
 * @Author: JohnJeep
 * @Date: 2021-01-24 17:08:19
 * @LastEditTime: 2021-01-24 17:31:04
 * @LastEditors: Please set LastEditors
 * @Description: 查看当前编译器使用的C++版本和GUN版本
 */
#include <iostream>
#include <cstring>

using namespace std;

int main(int argc, char *argv[])
{
    cout << __GNUC__ << endl;
    cout << __cplusplus << endl;
    
    return 0;
}