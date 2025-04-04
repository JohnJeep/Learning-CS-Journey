/*
 * @Author: JohnJeep
 * @Date: 2020-07-13 15:01:32
 * @LastEditTime: 2021-05-23 13:17:56
 * @LastEditors: Please set LastEditors
 * @Description: C++异常机制
 * 
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
    if (y == 0) {
        throw x;
    }

    return x / y; 
}


// 测试用例1
void test01()
{
    try {
        int result = divide(10, 0);
        // int result = divide(10, 2);
        cout << "result: " << result << endl;
    }
    catch(const std::exception& e) {
        std::cerr << e.what() << '\n';
    }
    catch(...) {
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
    try {
        except();
    }
    catch(int t) {
        cout << "int类型异常" << endl;
    }
    catch(...) {
        cout << "test02未知异常" << endl;
    }
}

int add(int x, int y)
{
    if (y == 0) {
        throw "y equal 0";
        // throw y;
    }

    return x / y;
}

// 处理普通的异常
void test03()
{
    try {
        int ret = add(4, 2);
        cout << "ret = " << ret << endl;
        int re = add(4, 0);
        cout << "re = " << re << endl;
    }
    catch (int e) {   // 捕获的类型由throw后面表达式的内容决定
        cout << e << endl;
    }
    catch(const char* e) {
        cout << e << endl;
    }
    catch (...) {
        cout << "execute ..." << endl;
    }
}

// 在继承中使用异常
struct MyStruct : public exception 
{
    const char* what() const throw()
    {
        return "C++ exception";
    }
};

void test04()
{
    try {
        throw MyStruct();
    }
    catch(MyStruct& e) {
        cout << "catch MyStruct" << endl;

        // what() 是异常类提供的一个公共方法，它已被所有子异常类重载，这是返回异常产生的原因。
        cout << e.what() << endl;  
    }
    
}

int main(int argc, char *argv[])
{
    test01();
    test02();
    test03();
    test04();

    return 0;
}