/*
 * @Author: your name
 * @Date: 2021-09-14 17:53:52
 * @LastEditTime: 2021-09-14 18:01:37
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Science-Journey\test.cpp
 */
#include <iostream>
using namespace std;

class test
{
private:
    /* data */
public:
    test(/* args */) {}
    ~test() {}

    void show() { cout << "Showing..." << endl; }
    static print() { cout << "Printting..." << endl; }
};


int main() 
{
    test t;
    test::print();
    t.show();
    return 0;
}