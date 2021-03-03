/*
 * @Author: JohnJeep
 * @Date: 2020-07-19 11:47:03
 * @LastEditTime: 2021-03-03 22:29:31
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库中map与multimap用法
 */ 
#include <iostream>
#include <cstdlib>
#include <string>
#include <map>

using namespace std;

// 测试用例01
void test01()
{
    cout << "test01 case!" << endl;
    // 四种map容器的插入方法
    map<int, string> mp;
    pair<map<int, string>::iterator, bool> tmp;

    mp.insert(pair<int, string>(101, "zhao"));                   // 法一
    mp.insert(make_pair<int, string>(102, "guan"));              // 法二
    tmp = mp.insert(make_pair<int, string>(102, "guan"));        // 法二
    if (tmp.second != true)
    {
        cout << "second insert failure." << endl;
    }
    else
    {
        cout << tmp.first->first << "\t" << tmp.first->second << endl;
    }
    mp.insert(map<int, string>::value_type(103, "cao"));           // 发三
    mp[104] = "zhangfei";                                          // 法四
    mp[104] = "zhang";                                             // 覆盖上次容器中键值的数据

    // 遍历
    map<int, string>::iterator it;
    cout << "map size: " <<mp.size() << endl;
    cout << "map element: " << endl;
    for (it = mp.begin(); it != mp.end(); it++)
    {
        cout << it->first << "\t" << it->second << endl;
    }
    cout << endl;
    
    // 删除
    cout << "map element delete: " << endl;
    while(!mp.empty())
    {
        it = mp.begin();
        cout << it->first << "\t" << it->second << endl;
        mp.erase(it);
    }
    cout << "map delete element size: " <<mp.size() << endl;
    cout << endl;
}

// multimap 测试用例
class Person
{
private:
    string m_name;
    int m_age;
    double m_salary;
public:
    Person(string name, int age, double salary);
    ~Person();
    string getName() const {return m_name;}
    int getAge() const {return m_age;}
    double getSalary() const {return m_salary;}
    void setSalary(double salary);
};

Person::Person(string name, int age, double salary)
{
    this->m_name = name;
    this->m_age = age;
    this->m_salary = salary;
}

Person::~Person()
{
}

void Person::setSalary(double salary)
{
    this->m_salary = salary;
}

void test02()
{
    cout << "test02 case!" << endl;
    Person p1("Jefe", 20, 12000), p2("peek", 23, 10000), p3("angle", 19, 14000);
    Person p4("pool", 22, 15000), p5("greek", 25, 17000);

    multimap<string, Person> mlp;
    mlp.insert(pair<string, Person>("sale", p1));
    mlp.insert(pair<string, Person>("sale", p2));
    mlp.insert(make_pair("development", p3));
    mlp.insert(make_pair("development", p4));
    mlp.insert(multimap<string, Person>::value_type("Finance", p5));

    multimap<string, Person>::iterator it;
    cout << "multimap size: " << mlp.size() << endl;
    cout << "multimap element: " << endl;
    for (it = mlp.begin(); it != mlp.end(); it++)
    {
        cout << it->first << "\t" 
             << it->second.getName() << "\t"
             << it->second.getAge() << "\t"
             << it->second.getSalary() << endl;
    }
    cout << endl;

    int num = mlp.count("development");
    cout << "development part number: " << num << endl;
    int tag = 0;
    it = mlp.find("development");
    while ((it != mlp.end()) && (tag < num))   // 遍历输出development部门的人员的信息 
    {
        cout << it->first << "\t" 
             << it->second.getName() << "\t"
             << it->second.getAge() << "\t"
             << it->second.getSalary() << endl;        
        it++;
        tag++;
    }
    cout << endl;

    // 按照条件进行检索，对检索到的值进行修改
    cout << "alter after multimap element:" << endl;
    for (it = mlp.begin(); it != mlp.end(); it++)
    {
        if (it->second.getSalary() == 14000)
        {
            it->second.setSalary(16000);
        }
        
        cout << it->first << "\t" 
             << it->second.getName() << "\t"
             << it->second.getAge() << "\t"
             << it->second.getSalary() << endl;
    }
}

int main(int argc, char *argv[])
{
    test01();
    test02();
    
    return 0;
}