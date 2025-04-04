/*
 * @Author: JohnJeep
 * @Date: 2021-05-25 20:33:28
 * @LastEditTime: 2021-05-25 23:57:59
 * @LastEditors: Please set LastEditors
 * @Description: unique_ptr 例子
 */
#include <iostream>
#include <cstdio>
#include <memory>
#include <string>
#include <functional>

using namespace std;

class Stu
{
private:
    int m_id;

public:
    Stu()
    {
        cout << "default constructor.." << endl;
    }
    Stu(int id) 
    {
        cout << "constructor, id = " << id << endl;
    }
    Stu(string name) 
    {
        cout << "costructor, name = " << name << endl;
    }
    ~Stu() 
    {
        cout << "destructor ..." << endl;
    }

    void setValue(int id)
    {
        m_id = id;
    }

    void getValue()
    {
        cout << "m_id = " << m_id << endl;
    }
};

// unique_ptr pointer
void test01()
{
    cout << "--------------------------------------" << endl;
    unique_ptr<string> up1(new string("I am unique pointer."));
    unique_ptr<string> up2;
    cout << *up1 << endl;
    // up2 = up1;      // 此时会报错！！编译不通过

    cout << "--------------------------------------" << endl;
    // 采用std::move()函数，up1将assignment的拥有权转移给up2，up1 对象不再拥有
    up2 = std::move(up1);   // assignment
    cout << *up2 << endl;

    cout << "--------------------------------------" << endl;
    unique_ptr<string> up3(std::move(up2));   // copy construct
    cout << *up3 << endl;

    cout << "--------------------------------------" << endl;
    unique_ptr<string> st;
    st.reset(new string("I am reset."));
    cout << *st << endl;

    // 将unique指针指向的值赋给一个临时对象temp_up
    // 临时对象的生命周期不应该太长，否则编译器会禁止。
    cout << "--------------------------------------" << endl;
    unique_ptr<string> temp_up = unique_ptr<string>(new string("I am temp object pointer."));
    cout << *temp_up << endl;
}

void test02()
{
    // 但lambda 表达式中没有写捕获参数时，要实现自己的删除器，需要在模板参数中指定其参数类型
    using func = void(*)(Stu*);    // void 类型的函数指针
    unique_ptr<Stu, func> s1(new Stu(100), [](Stu* p){    
        delete p;
    });


    // 有捕获参数时，unique_ptr 模板参数类型为 仿函数的返回类型
    unique_ptr<Stu, std::function<void (Stu*)>> s2(new Stu(200), [&](Stu* p){    
        delete p;
    });

    // 申请的内存为数组类型时，模板参数为数组类型
    unique_ptr<Stu[]> ptr1(new Stu[3]);

}


int main()
{
    // test01();
    test02();

    return 0;
}
