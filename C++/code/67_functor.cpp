/*
 * @Author: JohnJeep
 * @Date: 2021-05-12 23:37:17
 * @LastEditTime: 2021-05-12 23:49:02
 * @LastEditors: Please set LastEditors
 * @Description: 模拟实现仿函数机制
 */
#include <iostream>
#include <cstdlib>

using namespace std;

template<typename T>
struct fplus
{
    T operator() (const T& x, const T& y) const {
        return x + y;
    }
};

template<typename T>
struct fminus
{
    T operator() (const T& x, const T& y) const {
        return x - y;
    }
};



int main(int argc, char *argv[])
{
    fplus<int> plus_obj;
    fminus<int> minus_obj;

    cout << plus_obj(3, 5) << endl;
    cout << minus_obj(10, 3) <<endl;

    cout << fplus<int>()(5, 1) << endl;
    cout << fminus<int>()(100, 2) << endl;
    
    return 0;
}