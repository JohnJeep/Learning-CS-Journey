/*
 * @Author: your name
 * @Date: 2020-06-02 20:10:54
 * @LastEditTime: 2020-06-02 21:01:42
 * @LastEditors: Please set LastEditors
 * @Description: 简单的 C++ 语法测试
 * @FilePath: \Learning-Computer-Journey\C++\code\test.cpp
 */ 
#include <iostream>
using namespace std;

class People
{
private:
    const char* name;
    int age;

public:
    void eat()
    {
        cout << "I am eating" << endl;
    }

    People(/* args */);
    ~People();
};

People::People(/* args */)
{
    name = "Jackson";
    age = 50;
    cout << "I am constructor" << endl;
    printf("%s %d\n", name, age);
}

People::~People()
{
    cout << "I am destructor" << endl;
}


int main()
{
    People man;
    man.eat();

    return 0;
}