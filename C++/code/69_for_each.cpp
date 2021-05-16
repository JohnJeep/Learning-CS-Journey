/*
 * @Author: your name
 * @Date: 2021-05-15 18:23:25
 * @LastEditTime: 2021-05-16 13:43:35
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Science-Journey\69_for_each.cpp
 */
#include <iostream>
#include <cstdlib>
#include <vector>
#include <algorithm>

using namespace std;

class PrintInt
{
private:
    /* data */
public:
    PrintInt(/* args */) {}
    ~PrintInt() {}
    
    void operator() (int elem) const
    {
        cout << elem << " ";
    }
};

int main(int argc, char *argv[])
{
    vector<int> coll;

    for (int i=1; i<=9; ++i) {
        coll.push_back(i);
    }    

    for_each (coll.begin(), coll.end(), PrintInt()); // PrintInt() 是一个 function object
    cout << endl;
    
    return 0;
}