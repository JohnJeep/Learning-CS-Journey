/*
 * @Author: JohnJeep
 * @Date: 2021-05-07 19:12:43
 * @LastEditTime: 2021-05-24 15:45:45
 * @LastEditors: Please set LastEditors
 * @Description: 测试标准库中的 Array 容器
 */
#include <iostream>
#include <cstdlib>
#include <array>

using namespace std;

int main(int argc, char *argv[])
{
    array<int, 20> data;
    cout << data.at(6);
    cout << "hello" ;


    return 0;
}