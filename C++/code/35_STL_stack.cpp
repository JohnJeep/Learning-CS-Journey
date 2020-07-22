/*
 * @Author: JohnJeep
 * @Date: 2020-07-17 09:31:08
 * @LastEditTime: 2020-07-22 08:44:05
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库栈模型
 * @FilePath: /35_stl_stack.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <stack>

using namespace std;

void test01()
{
    stack<int> st;

    st.push(11);
    st.push(12);
    st.push(13);
    cout << "stack size: " << st.size() << endl;
    cout << "stack top: " << st.top() << endl;
    while (!st.empty())
    {
        int tmp = st.top();
        cout << "stack pop:" << tmp << endl;
        st.pop();
    }
}

// 栈里面存放的是类的对象类型
class Maid
{
private:
    int m_age;
public:
    Maid(int age);
    ~Maid();
    void show();
};

Maid::Maid(int age)
{
    this->m_age = age;
}

Maid::~Maid()
{
}
void Maid::show()
{
    cout << "maid age: " << m_age << endl;;
}

void test02()
{
    Maid smallWhilte(18);
    Maid smallBlue(20);
    Maid smallPure(22);
    
    stack<Maid> sm;
    sm.push(smallWhilte);
    sm.push(smallBlue);
    sm.push(smallPure);
    cout << "maid stack size: " << sm.size() << endl;

    while (!sm.empty())
    {
        Maid tp = sm.top();    // 获取当前栈的栈顶元素
        tp.show();      
        sm.pop();              // 要想栈为空，元素必须出栈
    }

    cout << endl;
    cout << "类对象为指针类型" << endl;
    Maid bunnyGirl(23);
    Maid butterfly(24);
    Maid lotus(25);
    Maid angle(26);
    
    stack<Maid*> spm;
    spm.push(&bunnyGirl);
    spm.push(&butterfly);
    spm.push(&lotus);
    spm.push(&angle);
    cout << "maid pointer stack size: " << spm.size() << endl;
    while (!spm.empty())
    {
        Maid *tpp = spm.top();    
        tpp->show();      
        spm.pop();              
    }

}

int main(int argc, char *argv[])
{
    test01();
    test02();
    return 0;
}