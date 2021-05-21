/*
 * @Author: JohnJeep
 * @Date: 2020-06-17 10:56:48
 * @LastEditTime: 2021-05-20 22:27:28
 * @LastEditors: Please set LastEditors
 * @Description: 派生模板类和普通类继承父类的模板类
 */ 
#include <iostream>
using namespace std;

template <typename T>
class Doctor
{
public:
    T m_prescribe;   // 药方
public:
    Doctor(T prescribe);
    ~Doctor();

    void show();
};

template <typename T>
Doctor<T>::Doctor(T prescribe)
{
    this->m_prescribe = prescribe;
}

template <typename T>
Doctor<T>::~Doctor()
{
}

template <typename T>
void Doctor<T>::show()
{
    cout << "m_prescribe: " << m_prescribe << endl;
}

// 普通的子类继承父类的模板类
class Nurse:public Doctor<int>
{
private:
    int m_age;
public:
    Nurse(int a);
    ~Nurse();
    void showAge()
    {
        cout << "m_age: " << m_age << endl;
    }
};

// template <typename T>  ginseng
Nurse::Nurse(int a)
:Doctor<int>(911)    // 显示初始化父类的参数列表，需要指定一个默认的值
{
    this->m_age = a;
}

Nurse::~Nurse()
{
}


// 模板类继承父类的模板类
template <typename T>
class Dentist:public Doctor<T>
{
private:
    int m_dt;
public:
    Dentist(int dt);
    ~Dentist();
    void display()
    {
        cout << "m_dt: " << m_dt << endl;
    }
};

template <typename T>
Dentist<T>::Dentist(int dt)
:Doctor<T>(1000)
{
    this->m_dt = dt;
}

template <typename T>
Dentist<T>::~Dentist()
{
}

int main()
{
    Doctor<int> wang(110);
    Nurse chen(30);
    chen.showAge();
    chen.show();   // 调用父类继承的方法

    //  模板类继承父类的模板类
    Dentist<int> zhao(666);
    zhao.display();
    zhao.show();

    return 0;
}


