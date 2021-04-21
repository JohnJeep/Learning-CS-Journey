/*
 * @Author: JohnJeep
 * @Date: 2021-04-21 23:00:21
 * @LastEditTime: 2021-04-22 00:24:14
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

enum class color
{
    red,
    green,
    yellow
};

enum class co 
{
    red,
};

int main(int argc, char *argv[])
{
    color col = color::yellow;

    switch (col)
    {
    case color::red:
        cout << "I am red.\n"; 
        break;
    case color::green:
        cout << "I am green.\n"; 
        break;
    case color::yellow:
        cout << "I am yellow.\n"; 
        break;
    
    default:
        cout << "I am default.\n";
        break;
    }    
    return 0;
}
