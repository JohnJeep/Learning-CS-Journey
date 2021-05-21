/*
 * @Author: JohnJeep
 * @Date: 2020-06-16 15:40:32
 * @LastEditTime: 2020-08-12 14:14:59
 * @LastEditors: Please set LastEditors
 * @Description: 单个类模板
 */ 
#include <iostream>
using namespace std;

template <typename T1, typename T2>
class Student
{
private:
    T1 name;
    T2 age;
public:
    Student(T1 m_name, T2 m_age);
    ~Student();

    void show()
    {
        cout << "name:" << name << endl;
        cout << "age:" << age << endl;
    }
};

// 类的外部实现类模板
template <typename T1, typename T2>
Student<T1, T2>::Student(T1 m_name, T2 m_age)
{
    this->name = m_name;
    this->age = m_age;
}

template <typename T1, typename T2>
Student<T1, T2>::~Student()
{
}


// 类模板做函数参数，其中Student<string, int>使用的类型要与实例化类时，声明的类型一样
void info(Student<string, int>& obj)
{
    obj.show();
}

int main()
{
    // Student<string, int> wang("王二", 20);
    Student<const char*, int> wang("王二", 20);
    Student<string, int> li("李大锤", 99);
    wang.show();

    // 调用类模板做函数参数
    info(li);

    return 0;
}


