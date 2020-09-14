/*
 * @Author: JohnJeep
 * @Date: 2020-09-14 10:05:54
 * @LastEditTime: 2020-09-14 16:16:47
 * @LastEditors: Please set LastEditors
 * @Description: 原型模式 
 */
#include <iostream>
#include <cstdio>

using namespace std;


class Person
{
private:
    /* data */
public:
    Person(/* args */);
    virtual ~Person();
    virtual void getInfo() = 0;
    virtual Person* Study() = 0;
};

Person::Person(/* args */)
{
}

Person::~Person()
{
}


class Stu : public Person
{
private:
    string m_name;
    int m_age;
public:
    // Stu() : m_name(" "), m_age(0)
    // {

    // }
    Stu() 
    {
        this->m_name = "";
        this->m_age = 0;
    }
    Stu(string name, int age) 
    {
        this->m_name = name;
        this->m_age = age;
    }
    ~Stu() 
    {

    }
    virtual void getInfo()
    {
        cout << "name: " << m_name << "\t" << "age: " << m_age << endl;
    }
    virtual Person* Study()
    {
        Stu* tmp = new Stu();
        *tmp = *this;
        return tmp;
    }
};

int main(int argc, char *argv[])
{
    Person* collageOne = new Stu("liyunlong", 35);
    collageOne->getInfo();

    // Person* collageTwo = collageOne;  // 执行浅拷贝
    Person* collageTwo = collageTwo->Study();  
    collageTwo->getInfo();

    return 0;
}