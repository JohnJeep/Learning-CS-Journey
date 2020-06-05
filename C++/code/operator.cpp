/*
 * @Author: JohnJeep
 * @Date: 2020-06-05 11:38:50
 * @LastEditTime: 2020-06-05 16:14:25
 * @LastEditors: Please set LastEditors
 * @Description: 操作运算符重载
 * @FilePath: /C++/code/operator.cpp
 */ 
#include <iostream>
using namespace std;

class Complex
{
private:
    int m_real; 
    int m_image; 
    friend Complex operator+(Complex& c1, Complex& c2);
    friend Complex operator++(Complex& cp, int);            // i++
    friend Complex& operator++(Complex& cp);                // ++i
    friend ostream& operator<<(ostream& os, Complex& cp);
public:
    Complex(const int real, const int image);
    ~Complex();
    Complex& operator-(Complex& cp);
    Complex operator--(int);                                // i--
    Complex& operator--();                                  // --i
};

Complex::Complex(const int real, const int image)
:m_real(real), m_image(image)
{
    cout << "执行构造函数" << endl;
}

Complex::~Complex()
{
}

// 全局函数重载操作符 +
Complex operator+(Complex& c1, Complex& c2)
// Complex& operator+ (const Complex& c1, const Complex& c2)
{
    return Complex(c1.m_real + c2.m_real, c1.m_image + c2.m_image);  // 标准库中临时对象的写法
    // Complex tmp(c1.m_real + c2.m_real, c1.m_image + c2.m_image);
    // return tmp;
}

// 全局函数重载操作符 i++
Complex operator++(Complex& cp, int)    // 添加 int 类型，做函数重载
{
    Complex tmp = cp;   // 临时对象先将 cp 对象拷贝一份，在进行后面运算
    cp.m_real++;
    cp.m_image++;
    return tmp;
}

// 全局函数重载操作符 ++i
Complex& operator++(Complex& cp)
{
    cp.m_real++;
    cp.m_image++;
    return cp;
}

/**
 * @description: 成员函数重载操作运算符 -，
 *               将当前对象数据减去cp对象的数据的结果赋值给当前对象
 * @param {type} 
 * @return:      对象的引用
 */
// 
Complex& Complex::operator-(Complex& cp)
{
    this->m_real -= cp.m_real;
    this->m_image -= cp.m_image;
    // cp.m_real = this->m_real - cp.m_real;
    // cp.m_image = this->m_image - cp.m_image;
    return *this;
}

// 成员函数重载操作运算符 i--
Complex Complex::operator--(int) // i--
{
    Complex tmp = *this;
    this->m_real--;
    this->m_image--;
    return tmp;
}

// 成员函数重载操作运算符 --i
Complex& Complex::operator--()
{
    this->m_real--;
    this->m_image--;
    return *this;     // 返回当前变量的值   
}

// 成员函数不能重载 << 操作运算符，返回值得不到 ostream& 引用的值
// ostream& Complex::operator>> (Complex& cp)
// {
//     return cp.m_real >> "+" >> cp.m_image >> "i";
// } 

// 全局函数重载 << 操作运算符
ostream& operator<< (ostream& os, Complex& cp)
{
    return os << cp.m_real << "+" << cp.m_image << "i";
}

int main()
{
    Complex cp1(11, 22);
    cout << "cp1: " << cp1 <<endl;
    Complex cp2(33, 44);
    cout << "cp2: " << cp2 <<endl;
    Complex cp3 = cp1 + cp2;
    cout << "cp3: " << cp3 <<endl;
    Complex cp4 = cp1++;
    cout << "cp4: " << cp4 <<endl;
    Complex cp5 = ++cp2;
    cout << "cp5: " << cp5 <<endl;


    Complex csub1(10, 20);
    cout << "csub1: " << csub1 <<endl;  // 成员函数中的结果改变了 csub1 对象的值
    Complex csub2(30, 40);
    cout << "csub2: " << csub2 <<endl;
    Complex csub3 = csub2 - csub1;
    cout << "csub3: " << csub3 <<endl;
    Complex csub4 = csub1--;
    cout << "csub4: " << csub4 <<endl;
    Complex csub5 = --csub2;
    cout << "csub5: " << csub5 <<endl;
    
    system("pause");
    return 0;
}

