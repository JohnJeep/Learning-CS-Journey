#include <iostream>
using namespace std;

class base
{
private:
public:
    base() {
        cout << "Base." << endl;
    }
    void f1() {
        cout << "base f1 called." << endl;
    }
    virtual void f2() {
        cout << "base f2 called." << endl;
    }

    void show (int x) {
        cout << "Base show called with int: " << x << endl;
    }

    ~base() {
        cout << "Base destructor." << endl;
    }
};

class derived : public base
{
private:
public:
    derived(/* args */) {
        cout << "Derived." << endl;
    }
    void f1() {
        cout << "derived f1 called." << endl;
    }
    void f2() override {
        cout << "derived f2 called." << endl;
    }
    void show(int x) {
        cout << "Derived show called with int: " << x << endl;
    }  
    ~derived() {
        cout << "Derived destructor." << endl;
    }
};


int main(int argc, char const *argv[])
{
    base* bptr = new base();
    base* dptr = new derived();

    bptr->f1();
    bptr->f2();

    dptr->f1(); // 调用父类。非虚函数在编译时根据指针的静态类型（Base*）决定调用哪个函数。
    dptr->f2(); // 调用子类。虚函数在运行时根据指针指向的实际对象类型（Derived）决定调用哪个函数。

    derived* dptr2 = new derived();
    dptr2->f1();
    dptr2->f2();

    delete bptr;
    delete dptr;

    return 0;
}