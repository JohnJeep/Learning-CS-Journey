/*
 * @Author: your name
 * @Date: 2020-06-10 14:16:58
 * @LastEditTime: 2020-06-10 15:04:19
 * @LastEditors: Please set LastEditors
 * @Description: 虚函数中使用多个继承来模拟接口的方式
 *               子类初级工程师和高级工程师分别继承与父类工程师，都在子类中重写了一个虚函数
 *               在已有的接口上，新增一个架构师类，来继承父类的方法
 * @FilePath: /C++/code/multi_virtual.cpp
 */ 

#include <iostream>
using namespace std;

class Engineer
{
public:

public:
    Engineer();
    ~Engineer();
    virtual void getWage() = 0;  // 纯虚函数在父类中声明，子类中去实现
};

Engineer::Engineer()
{
}

Engineer::~Engineer()
{
}


class JuniorEngineer:public Engineer
{
private:
    const char *j_name;
    double j_salary;
public:
    JuniorEngineer(const char *name, double salary);
    ~JuniorEngineer();

    virtual void getWage()
    {
        cout << "初级工程师: " << j_name << "\t" << "薪水：" << j_salary << endl;
    } 
};

JuniorEngineer::JuniorEngineer(const char *name, double salary)
{
    this->j_name = name;
    this->j_salary = salary;
}

JuniorEngineer::~JuniorEngineer()
{
}


class SeniorEnginner:public Engineer
{
private:
    const char *s_name;
    double s_salary;
public:
    SeniorEnginner(const char *name, double salary);
    ~SeniorEnginner();
    virtual void getWage()
    {
        cout << "高级工程师: " << s_name << "\t" << "薪水：" << s_salary << endl;
    } 
};

SeniorEnginner::SeniorEnginner(const char *name, double salary)
{
    this->s_name = name;
    this->s_salary = salary;
}

SeniorEnginner::~SeniorEnginner()
{
}


/************向已有的 computeWage 接口中新增一个架构师类**********************/
class Architect:public Engineer
{
private:
    const char *a_name;
    double a_salary;
public:
    Architect(const char *name, double salary);
    ~Architect();
    virtual void getWage()
    {
        cout << "架构师: " << a_name << "\t" << "薪水：" << a_salary << endl;
    }    
};

Architect::Architect(const char *name, double salary)
{
    this->a_name = name;
    this->a_salary = salary;
}

Architect::~Architect()
{
}


// 父类的引用指向子类的对象
void computeWage(Engineer& obj)
{
    obj.getWage();
}

int main()
{

    JuniorEngineer J1("赵子龙", 5000);
    SeniorEnginner S1("王二麻子", 12000);
    computeWage(J1);
    computeWage(S1);

    // 新增的接口
    Architect A1("子非鱼", 30000);
    computeWage(A1);
    return 0;
}
