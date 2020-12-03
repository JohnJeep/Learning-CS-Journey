/*
 * @Author: JohnJeep
 * @Date: 2020-12-02 15:10:42
 * @LastEditTime: 2020-12-03 19:20:06
 * @LastEditors: Please set LastEditors
 * @Description: protect在继承中的使用
 */
#include <iostream>
#include <cstdio>

using namespace std;

class Father
{
private:
    int f_data;
    void func1() { cout << "我是父类private方法" << endl;}
protected:
    int f_p;
    void func2() { cout << "我是父类protect方法" << endl;}

public:
    int f_t;
    void func3() { cout << "我是父类public方法" << endl;}

public:
    Father(/* args */) {}
    ~Father() {}
};


class Child : public Father
{
private:
    int c_data;
    void cfunc1() { cout << "我是子类private方法" << endl;}

protected: 
    int c_p;
    void cfunc2() { cout << "我是子类protect方法" << endl;}

public:
    int c_t;
    void cfunc3() { cout << "我是子类public方法" << endl;}

public:
    Child(/* args */) {
        this->cfunc2();
        this->func2();  // 访问父类的protected 方法
    }
    ~Child() {}
};



int main(int argc, char *argv[])
{
    Child lilong;
    Father gang;

    
    lilong.cfunc3();
    lilong.func3();
    
    gang.func3();

    return 0;
}