/*
 * @Author: JohnJeep 
 * @Date: 2021-04-29 22:30:59
 * @LastEditTime: 2021-05-24 22:55:02
 * @LastEditors: Please set LastEditors
 * @Description: 初始化列表
 */
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <initializer_list>
#include <vector>

using namespace std;

class Money
{
private:
    vector<int> m_vec;
public:
    Money(initializer_list<int> num) 
    {
        cout << "Test initialier_list..." << endl;
        for (const auto& it : num) {
            cout << it << " ";
            m_vec.push_back(it);
            it++;
        }
        cout << endl;
    }
    ~Money() {}
};

int main(int argc, char *argv[])
{
    // int a = {3.14};    // error: 编译器不通过，使用 {} 之后，不能隐式的进行类型转换
    int b = 3.14;      // right: 编译通过，进行隐式的类型转

    cout << "Cpmare multi nums max: "<< max({100, 99, 34, 67, 868, 344}) << endl;
    cout << "Compare two nums max: " << max(100, 34) << endl;

    Money RMB({1, 5, 10, 100});   // 通过 initializer_list 进行初始化

    return 0;
}