/*
 * @Author: JohnJeep
 * @Date: 2021-05-23 11:36:19
 * @LastEditTime: 2021-05-23 12:10:40
 * @LastEditors: Please set LastEditors
 * @Description: 条件表达式用法
 */
#include <iostream>
#include <cstdlib>

using namespace std;

int main(int argc, char *argv[])
{
    cout << "Please input number: ";
    int num;
    cin >> num;

    num = num > 100 ? num : 222;
    cout << num << endl;
    
    return 0;
}