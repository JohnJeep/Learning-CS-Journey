/*
 * @Author: JohnJeep
 * @Date: 2020-09-16 08:57:39
 * @LastEditTime: 2020-09-16 09:29:15
 * @LastEditors: Please set LastEditors
 * @Description: 外观模式
 *               描述：
 */
#include <iostream>
#include <cstdio>

using namespace std;

class SystemA
{
private:
    /* data */
public:
    SystemA(/* args */) {}
    ~SystemA() {}
    void work()
    {
        cout << "systemA work" << endl;
    }
};

class SystemB
{
private:
    /* data */
public:
    SystemB(/* args */) {}
    ~SystemB() {}
    void work()
    {
        cout << "systemB work" << endl;
    }
};

class SystemC
{
private:
    /* data */
public:
    SystemC(/* args */) {}
    ~SystemC() {}
    void work()
    {
        cout << "systemC work" << endl;
    }
};

// 内部提供了一层接口，调用者使用Facade类操作SystemA、SystemB、SystemC，
// 而不是直接去操作下层的类，提供了一个良好的接口，方便可扩展。
class Facade
{
private:
    SystemA* sa;
    SystemB* sb;
    SystemC* sc;
public:
    Facade() 
    {
        sa = new SystemA;
        sb = new SystemB;
        sc = new SystemC;
    }
    ~Facade() 
    {
        delete sa;
        delete sb;
        delete sc;
    }
    void doSomething()
    {
        sa->work();
        sb->work();
        sc->work();
    }
};

int main(int argc, char *argv[])
{
    Facade* facd = new Facade;
    facd->doSomething();
    delete facd;

    return 0;
}