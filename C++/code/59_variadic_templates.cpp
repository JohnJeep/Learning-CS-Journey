/*
 * @Author: JohnJeep
 * @Date: 2021-04-23 22:27:53
 * @LastEditTime: 2021-04-25 23:24:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <bitset>

using namespace std;

/**
 * @description: 当包 ... 中 的个数等于0时就会执行下面这个空的函数 
 * @param {*}
 * @return {*}
 */
void my_print()
{
}

template<typename T, typename... Types>
void my_print(const T& firstAgs, const Types&... args)
{
    cout << firstAgs << endl;
    my_print(args...);
}

int main(int argc, char *argv[])
{
    my_print(100, "hello", bitset<16>(377), 50);

    return 0;
}
