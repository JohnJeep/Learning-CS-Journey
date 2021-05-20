/*
 * @Author: JohnJeep
 * @Date: 2021-05-05 13:27:55
 * @LastEditTime: 2021-05-05 14:13:54
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
#include <iostream>
#include <cstdlib>
#include <vector>

using namespace std;

void process(int& t)
{   
    cout << "process(int& t): " << t << endl;
}

void process(int&& t)
{
    cout << "process(int&& t): " << t << endl;
}

void forward(int&& t)
{
    cout << "forward(int&& t): " << t << endl;
    process(t);
}

int main(int argc, char *argv[])
{
    int a = 100;

    process(a);  // 参数被当做左值处理
    process(1);  // temp.object 被当做右值处理
    process(std::move(a));  // 强制将变量改为右值处理

    forward(2); // 右值经过 forward() 后变为左值，因为它在传递过程中变为了一个 name object
    forward(std::move(a)); // 结果变为左值

    // forward(a) // 参数出为左值，编译 error
    
        
    return 0;
}