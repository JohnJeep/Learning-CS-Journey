/*
 * @Author: JohnJeep
 * @Date: 2020-09-17 16:10:52
 * @LastEditTime: 2020-09-17 16:19:05
 * @LastEditors: Please set LastEditors
 * @Description: 备忘录模式
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Person
{
private:
    string m_name;
    int m_age;
public:
    Person(string name, int age) 
        : m_name(name), m_age(age)
    {}
    ~Person() {}
};

class Memo
{
private:
    /* data */
public:
    Memo(/* args */) {}
    ~Memo() {}
};
int main(int argc, char *argv[])
{
    
    return 0;
}