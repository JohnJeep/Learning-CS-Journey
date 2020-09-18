/*
 * @Author: JohnJeep
 * @Date: 2020-09-17 16:10:52
 * @LastEditTime: 2020-09-18 14:44:53
 * @LastEditors: Please set LastEditors
 * @Description: 备忘录模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Memo
{
private:
    string s_name;
    int s_age;
public:
    Memo(string name, int age) 
        : s_name(name), s_age(age) 
    {}
    ~Memo() 
    {}

    string getName() const {return s_name;}
    int getAge() const {return s_age;}
};

class Person
{
private:
    string m_name;
    int m_age;
public:
    Person(string name, int age) 
        : m_name(name), m_age(age)
    {}
    ~Person() 
    {}

    void setName(string name) {this->m_name = name;}
    void setAge(int age) {this->m_age = age;}
    string getName() const {return m_name;}
    int getAge() const {return m_age;}
    Memo* saveMemo()
    {
        return new Memo(m_name, m_age);
    }
    void recoverMemo(Memo* mo)
    {
        this->m_name = mo->getName();
        this->m_age = mo->getAge();
    }
    void showInfo()
    {
        cout << "name:" << m_name << "\t" << "age: " << m_age << endl;
    }
};

int main(int argc, char *argv[])
{
    Person* p = new Person("Jade", 24);
    p->showInfo();
    
    Memo* onenote = p->saveMemo();  // 将Person对象中的数据保存到备忘录对象中
    p->setAge(30);                  // 改变Person对象中的数据
    p->setName("Anna");
    p->showInfo();
    
    p->recoverMemo(onenote);        // 从备忘录对象中恢复数据 
    p->showInfo();
    delete p;
    delete onenote;

    return 0;
}