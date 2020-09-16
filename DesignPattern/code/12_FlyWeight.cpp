/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 09:29:57
 * @LastEditTime: 2020-09-16 11:34:00
 * @LastEditors: Please set LastEditors
 * @Description: 享元模式的实现
 * 
 */
#include <iostream>
#include <cstdio>
#include <string>
#include <map>

using namespace std;

class Person
{
protected:
    string m_name;
    int m_age;
public:
    Person(string name, int age) : m_name(name), m_age(age)
    {

    }
    virtual ~Person() {}
    virtual void getInfo() = 0;
};

class Teacher : public Person
{
private:
    int m_id;
public:
    Teacher(string t_name, int t_age, int t_id); 
    ~Teacher();
    virtual void getInfo()
    {
        cout << "name: " << m_name << "\t" 
             << "age: "  << m_age  << "\t"
             << "id: "   << m_id   << endl;
    }       
};

Teacher::Teacher(string t_name, int t_age, int t_id) 
    : Person(t_name, t_age), m_id(t_id)
{

}

Teacher::~Teacher()
{

}

class FlyWeight
{ 
private:
    map<int, Person*> mp;    
public:
    FlyWeight() 
    {
        mp.clear();
    }
    ~FlyWeight() 
    {
        // 进行内存的释放
        while (!mp.empty())
        {
            Person* p = nullptr;
            map<int, Person*>::iterator it = mp.begin();
            p = it->second;
            mp.erase(it);  // 从容器中删除一个结点
            delete p;
        }
    }
    Person* getTeacherId(int id);
};

// 通过ID号去查找容器中是否有老师的结点
Person* FlyWeight::getTeacherId(int id)
{
    map<int, Person*>::iterator iter;
    Person* tmp = nullptr;
    iter = mp.find(id);
    if (iter == mp.end())  // 容器中没有老师的ID
    {
        string name;
        int age;

        cout << "Please input name: ";
        cin >> name;
        cout << endl;
        cout << "Please input age: ";
        cin >> age;
        cout << endl;

        tmp = new Teacher(name, age, id);
        mp.insert(pair<int, Person*>(id, tmp));
    }
    else
    {
        tmp = iter->second;   // 迭代器的第二个值为Person对象
    }
    return tmp;
}



int main(int argc, char *argv[])
{
    Person* p1 = nullptr;
    Person* p2 = nullptr;

    // p1 = new Teacher("张三", 40, 11);   // 一般使用的方法好构建类的对象
    // p2 = new Teacher("李四", 37, 22);

    FlyWeight* flw = new FlyWeight;   // 在FlyWeight类里面去创建新的对象
    p1 = flw->getTeacherId(11);
    p1->getInfo();

    p2 = flw->getTeacherId(11);  // 利用享元类共享容器中已存在的对象
    p2->getInfo();

    delete p1;
    delete p2;

    return 0;
}