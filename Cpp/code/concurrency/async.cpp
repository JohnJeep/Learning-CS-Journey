/*
 * @Author: JohnJeep
 * @Date: 2021-08-08 01:46:33
 * @LastEditTime: 2021-08-08 02:17:58
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
 */
#include <future>
#include <iostream>
#include <chrono>
#include <random>
#include <thread>
#include <exception>

using namespace std;

int doSomething(char c)
{
    std::default_random_engine dre(c);
    std::uniform_int_distribution<int> id(10, 1000);

    for (int i = 0; i < 10; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(id(dre)));
        std::cout.put(c).flush();
    }

    return c;
}

int func1()
{
    return doSomething('.');
}

int func2()
{
    return doSomething('+');
}

int main()
{
    std::cout << "starting func1() background"
              << "starting func2() foreground: " << std::endl;
    
    // std::future<int> result1(std::async(func1));
    // std::future<int> result1 = std::async(std::launch::async, func1);
    int result2 = func2();

    int result = result1.get() + result2;
    std::cout << "\nresult of func1() + func2(): " <<result << std::endl;
    
    return 0;
}