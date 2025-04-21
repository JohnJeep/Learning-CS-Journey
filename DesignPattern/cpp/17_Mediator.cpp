/*
 * @Author: JohnJeep
 * @Date: 2020-09-17 10:35:48
 * @LastEditTime: 2020-09-17 15:32:08
 * @LastEditors: Please set LastEditors
 * @Description: 中介模式实现
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Agency;
class Person
{
protected:
    string m_name;
    int m_age;
    string m_sex;
    Agency* m_agency;
public:
    Person(string name, int age, string sex, Agency* agency)
        : m_name(name), m_age(age), m_sex(sex), m_agency(agency) 
    {}
    virtual ~Person() 
    {}

    string getName() const {return m_name;}
    int getAge() const {return m_age;}
    string getSex() const {return m_sex;}
    virtual void matchObject(Person* p) = 0;  
};

// 中介者中持有所要执行对象的引用
class Agency
{
private:
    Person* m_boy;
    Person* m_giral;
public:
    Agency() 
    {}
    ~Agency() {}
    void setBoy(Person* boy)
    {
        this->m_boy = boy;
    }
    void setGiral(Person* giral)
    {
        this->m_giral = giral;
    }

    // 通过性别和年龄来匹配
    void dealMatch()  
    {
        if (m_boy->getSex() == m_giral->getSex())
        {
            cout << "Sex not match" << endl;
        }
        else if (m_boy->getAge() == m_giral->getAge())
        {
            cout << "Match well" << endl;
        }
        else
        {
            cout << "We are not match" << endl;
        }
    }
};


class Boy : public Person
{
private:

public:
    Boy(string b_name, int b_age, string b_sex, Agency* b_agency)
        : Person(b_name, b_age, b_sex, b_agency)   // 初始化父类的默认构造
    {}
    ~Boy() 
    {}
    void matchObject(Person* p)
    {
        m_agency->setBoy(this);
        m_agency->setGiral(p);
        m_agency->dealMatch();
    }
};

class Giral : public Person
{
private:
    /* data */
public:
    Giral(string g_name, int g_age, string g_sex, Agency* g_agency)
        : Person(g_name, g_age, g_sex, g_agency)
    {}
    ~Giral() 
    {}
    void matchObject(Person* p) 
    {
        m_agency->setBoy(p);
        m_agency->setGiral(this);
        m_agency->dealMatch();      // 利用中介的方式去处理两个对象 
    }
};

int main(int argc, char *argv[])
{
    Agency* Zhengai = new Agency;
    Person* jiu = new Boy("Scott", 25, "man", Zhengai);
    Person* qin = new Giral("Jade", 23, "woman", Zhengai);
    Person* xin = new Giral("xin", 25, "woman", Zhengai);
    jiu->matchObject(qin);    // 男孩去找女孩，
    jiu->matchObject(xin);
    delete xin;
    delete qin;
    delete jiu;
    delete Zhengai;

    return 0;
}