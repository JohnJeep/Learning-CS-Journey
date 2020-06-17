/*
 * @Author: JohnJeep
 * @Date: 2020-06-17 15:35:43
 * @LastEditTime: 2020-06-17 16:19:49
 * @LastEditors: Please set LastEditors
 * @Description: 类模板中的操作符运算操作
 * @FilePath: /C++/code/class_template_operator.cpp
 */ 
#include <iostream>
using namespace std;

template <typename T>
class Complex
{
public:
    T m_real;
    T m_img;

public:
    Complex(T real, T img);
    ~Complex();
    Complex<T> operator+(Complex<T>& cp);
    friend ostream& operator<< <T>(ostream& out, Complex<T>& obj);
};

template <typename T>
Complex<T>::Complex(T real, T img)
{
    this->m_real = real;
    this->m_img = img;
}

template <typename T>
Complex<T>::~Complex()
{
}

// 重载operator+ 运算符，重载后当前对象的值将发生改变
template <typename T>
Complex<T> Complex<T>::operator+(Complex<T>& cp)
{
    // this->m_real += cp.m_real;
    // this->m_img  += cp.m_img;
    // return *this;
    return Complex<T>(m_real + cp.m_real, m_img + cp.m_img); 
}

// 重载operator<< 运算符
template <typename T>
ostream& operator<<(ostream& out, Complex<T>& obj)
{
    return out << obj.m_real << obj.m_img << endl;
}


int main()
{
    Complex<int> c1(11, 22);
    Complex<int> c2(33, 44);
    Complex<int> c3 = c1 + c2;

    cout << "c1: " << c1 << endl;
    cout << "c2: " << c2 << endl;
    cout << "c3: " << c3 << endl;


    return 0;
}

