/*
 * @Author: JohnJeep
 * @Date: 2020-07-13 15:01:32
 * @LastEditTime: 2020-07-13 15:47:17
 * @LastEditors: Please set LastEditors
 * @Description: C++异常机制
 * @FilePath: /C++/code/29_try_catch.cpp
 */ 
#include <iostream>
using namespace std;


class Stu
{
private:
    int m_id;
    int m_key;
public:
    Stu(int id, int key);
    ~Stu();
};

Stu::Stu(int id, int key)
{
    this->m_id = id;
    this->m_key = key;
}

Stu::~Stu()
{
    cout << "执行析构函数" << endl;
}


int divide(int x, int y)
{
    if (y == 0)
    {
        throw x;
    }
    return x / y; 
}


// 测试用例1
void test01()
{
    try
    {
        int result = divide(10, 0);
        // int result = divide(10, 2);
        cout << "result: " << result << endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    catch(...)
    {
        cout << "test01未知异常" << endl;
    }

}

// 测试用例2
void except()
{
    Stu wang(007, 100);
    cout << "执行异常" << endl;
    throw 2;    // 抛出异常后类中的变量被析构了，内存空间被释放  
}

void test02()
{
    try
    {
        except();
    }
    catch(int t)
    {
        cout << "int类型异常" << endl;
    }
    catch(...)
    {
        cout << "test02未知异常" << endl;
    }
}


int main(int argc, char *argv[])
{
    test01();
    test02();
    return 0;
}