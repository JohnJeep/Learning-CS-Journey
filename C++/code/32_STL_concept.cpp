/*
 * @Author: Jonjeep
 * @Date: 2020-07-15 11:15:03
 * @LastEditTime: 2020-07-15 14:26:57
 * @LastEditors: Please set LastEditors
 * @Description: STL容器的基础概念
 * @FilePath: /32_stl_concept.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <vector>
#include <algorithm>

using namespace std;

class Teacher
{
public:
    int m_age;

public:
    Teacher(/* args */);
    ~Teacher();
    void showAge();
};

Teacher::Teacher(/* args */)
{
}

Teacher::~Teacher()
{
}
void Teacher::showAge()
{
    cout << "age: "<< m_age << endl;
}

ostream& operator<< (ostream& out, Teacher& te)
{
    return out << te.m_age;
}

void test01()
{
    vector<int> vt1;
    vt1.push_back(2);
    vt1.push_back(4);
    vt1.push_back(4);
    vt1.push_back(8);
    vt1.push_back(10);
    
    for (vector<int>::iterator it = vt1.begin(); it != vt1.end(); it++)
    {
        cout << "it: " << *it << endl;
    }
    int num = count(vt1.begin(), vt1.end(), 4);  // count()函数，计数迭代器中从开始到结束数据中指定数据出现的次数
    cout << "num: " << num << endl;    
}

void test02()
{
    Teacher wang, zhang, li;
    wang.m_age = 21;
    zhang.m_age = 23;
    li.m_age = 25;

    vector<Teacher> v2;
    v2.push_back(wang);    // 传递的是元素
    v2.push_back(zhang); 
    v2.push_back(li);

    for (vector<Teacher>::iterator it = v2.begin(); it != v2.end(); it++)
    {
        cout << "传递元素teacher it: " << it->m_age << endl;
    }
    cout << endl;
}

// 利用指针实现的第一种方式
void test03()
{
    Teacher wang, zhang, li;
    wang.m_age = 21;
    zhang.m_age = 23;
    li.m_age = 25;

    Teacher *pw, *pz, *pl;
    pw = &wang;
    pz = &zhang;
    pl = &li;

    vector<Teacher*> v2;
    v2.push_back(pw);    // 传递的是指针
    v2.push_back(pz); 
    v2.push_back(pl);

    for (vector<Teacher*>::iterator it = v2.begin(); it != v2.end(); it++)
    {
        cout << "传递指针teacher it: " << (*it)->m_age << endl;
    }
    cout << endl;
}

// 利用指针实现的第二种方式
void test04()
{
    // Teacher *pw;     //error，没有给对象分配内存空间
    Teacher *pw = new Teacher();
    Teacher *pz = new Teacher();
    Teacher *pl = new Teacher();
    pw->m_age = 21;
    pz->m_age = 23;
    pl->m_age = 25;

    vector<Teacher*> v2;
    v2.push_back(pw);    // 传递的是指针
    v2.push_back(pz); 
    v2.push_back(pl);

    for (vector<Teacher*>::iterator it = v2.begin(); it != v2.end(); it++)
    {
        cout << "传递指针teacher it: " << (*it)->m_age << endl;
    }
    delete pw;
    delete pz;
    delete pl;
}

int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    test04();
    return 0;
}
