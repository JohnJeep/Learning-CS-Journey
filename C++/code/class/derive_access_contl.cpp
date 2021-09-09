/*
 * @Baseuthor: JohnJeep
 * @Date: 2020-06-08 10:44:52
 * @LastEditTime: 2021-09-09 16:18:53
 * @LastEditors: Please set LastEditors
 * @Description: 派生类访问控制权限分析
 *  https://www.programiz.com/cpp-programming/public-protected-private-inheritance#:~:text=protected%20inheritance%20makes%20the%20public%20and%20protected%20members,base%20class%20are%20inaccessible%20to%20the%20derived%20class.
 * https://stackoverflow.com/questions/860339/difference-between-private-public-and-protected-inheritance
 */
#include <iostream>
using namespace std;

class Base {
private:
    int m_a;

protected:
    int m_b;

public:
    int m_c;

public:
    Base() {}
    ~Base() {}

    void set(int a, int b, int c) {
        cout << "set Base class " << endl;
        this->m_a = a;
        this->m_b = b;
        this->m_c = c;
    }

	// function to access private member
	int getPvt() {
		return m_a;
	}
};

class PublicDerived : public Base {
private:
    /* data */
public:
    void showB() {
        cout << "show B class" << endl;
        // cout << "m_a:" << m_a << endl;  // m_a is private
        cout << "m_b:" << m_b << endl;
        cout << "m_c:" << m_c << endl;
    }
    PublicDerived() {}
    ~PublicDerived() {}
	
	// function to access protected member from Base
	int getProtect() {
		return 
	}
};

class ProtectedDerived : protected Base {
private:
    /* data */
public:
    void showC()
    {
        cout << "show ProtectedDerived class" << endl;
        // cout << "m_a:" << m_a << endl;  // private 私有成员不能访问
        cout << "m_b:" << m_b << endl; // protected
        cout << "m_c:" << m_c << endl; // protected
    }
    ProtectedDerived() {}
    ~ProtectedDerived() {}

};

class PrivateDerived : private Base {
private:
    /* data */
public:
    void showD()
    {
        cout << "show PrivateDerived class" << endl;
        // cout << "m_a:" << m_a << endl;  // m_a is private 
        cout << "m_b:" << m_b << endl;     // m_b is private
        cout << "m_c:" << m_c << endl;     // m_c is private
    }
    PrivateDerived(/* args */);
    ~PrivateDerived();
};

int main()
{
    PublicDerived b1;
    b1.set(11, 22, 33);
    b1.m_c = 100; // 类的外部能调用子类 public属性的成员变量
    b1.showB();

    ProtectedDerived c1;
    // c1.m_c = 200;         // 类的外部能调用子类 protected 属性的成员变量
    // c1.set(44, 55, 66);   // set() 函数属性变为 protected，不能在类的外部使用
    c1.showC(); // 先调用 Base类的构造函数，再调用子类（C类）的构造
                // 先调用子类（ C类）的析构函数，再调用父类（Base类）的析构函数

    PrivateDerived d1;
    //   c1.m_c = 300; // 类的外部不能调用子类 private 属性的成员变量
    // d1.set();
    d1.showD();

    return 0;
}
