/*
 * @Author: JohnJeep
 * @Date: 2020-08-27 14:49:37
 * @LastEditTime: 2020-08-27 14:57:51
 * @LastEditors: Please set LastEditors
 * @Description: unordered_set 容器的使用
 */
#include <iostream>
#include <cstdio>
#include <unordered_set>

using namespace std;

int main(int argc, char *argv[])
{
    
    unordered_set<string> uset;

    uset.emplace("I am a unordered set.");
    uset.emplace("Are you really?");
    uset.emplace("Good?");
    uset.insert("I use insert.");

    cout << "uset size: " << uset.size() << endl;
    unordered_set<string>::iterator it;
    for (it = uset.begin(); it != uset.end(); it++)
    {
        cout << *it << "\n";
    }

    return 0;
}