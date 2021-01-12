/*
 * @Author: JohnJeep
 * @Date: 2021-01-11 09:48:55
 * @LastEditTime: 2021-01-11 10:10:24
 * @LastEditors: Please set LastEditors
 * @Description: override关键字
 */
#include <iostream>
using namespace std;

// 没有使用override
class Base
{
public:
	virtual void foo() { cout << "Base::foo" << endl; }
	virtual void goo() { cout << "Base::goo" << endl; }
	// ...
};

class Derived : public Base
{
public:
	void foo() { cout << "Derived::foo" << endl; }
	void gao() { cout << "Derived::goo" << endl; } // 错误的将goo写成了gao，但编译器并不会给出提示
	// ...
};

/*
// 使用override
class Base
{
public:
    virtual void foo() { cout << "Base::foo()" << endl;}
    virtual void bar() { cout << "Base::bar()" << endl;}
    void goo() { cout << "Base::goo()" << endl;}
};

class Derived : public Base
{
public:
    // ok
    void foo() override { cout << "Derived::foo()" << endl;}
    
    // error: Derived::foo does not override. 子类重写父类的函数，两者函数声明不一致
    void foo() const override { cout << "Derived::foo()" << endl;}

    // error: Base::goo is not virtual  
    void goo() override { cout << "Derived::goo()" << endl; }

    // error: 将bar误写成了bao，且基类中没有名为bao的虚函数
    // 由于使用了override，编译器会报错误
    void bao() override { cout << "Derived::bao()" << endl;}
};
*/

int main(void)
{
	Derived d;
	d.foo();                  // Derived::foo
	d.goo();                  // Base::goo 没有具体的子类实现，这不是我们想要的结果
	
	return 0;
}