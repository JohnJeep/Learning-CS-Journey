/*
 * @Author: JohnJeep
 * @Date: 2021-02-28 17:56:41
 * @LastEditTime: 2021-02-28 21:55:57
 * @LastEditors: Please set LastEditors
 * @Description: 构造函数中匿名对象的操作。匿名对象也叫临时对象(local object)。
 */
#include <iostream>
#include <stdlib.h>
#include <string>

using namespace std;

class Stu
{
public:
    Stu(string name, int id);          // constructor function
    Stu(const Stu& obj);               // copy constructor function
    ~Stu();                            // destructor function

    int getId() const 
    {
        cout << "id: " << m_id << endl;
        return m_id;
    }
    string getName() const 
    {
        cout << "name: " << m_name << endl;
        return m_name;
    }

private:
    string m_name;
    int    m_id;
};

Stu::Stu(string name, int id)
    : m_name(name), m_id(id)
{
    cout << "Execute constructor" << endl;
}

Stu::~Stu()
{
    cout << "Execute destructor" << endl;
    cout << "id: " << m_id << endl;
    cout << "name: " << m_name << endl;
}

Stu::Stu(const Stu& obj)
{
    m_id = obj.m_id;
    m_name = obj.m_name;
    cout << "Execute copy constructor" << endl;
}

// 函数的返回值是一个对象
Stu func1()
{
    // local variable
    Stu tmp("wang", 007);
    return tmp;                    

    // return Stu("wang", 007);   // create local object, equal to above the sentence
}

int main(int argc, char *argv[])
{
    func1();                        // 返回值是一个匿名对象，对象中的数据被析构

    // 用匿名对象去初始化 st 这个对象，C++编译器直接把匿名对象转化为新的有名对象，不被析构。
    Stu st = func1();                                                   
    st.getName();
    st.getId();

    // 用匿名对象去赋值给这个同类型的对象，匿名对象被析构。
    Stu art("li", 100);
    art = func1();     // art对象中原来的数据被func1返回对象中的数据覆盖
    
    return 0;
} 