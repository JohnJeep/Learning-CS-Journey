/*
 * @Author: JohnJeep 
 * @Date: 2022-05-11 21:12:02
 * @LastEditTime: 2022-05-11 21:21:23
 */
#include <iostream>
#include "integer.h"

using namespace std;

Integer::Integer() 
  : m_value(0)
{
    cout << "Execute default constructor." << endl;
}

Integer::Integer(int value) 
    : m_value(value)
{
    cout << "Execute constructor with parameter." << endl;
} 

Integer Integer::operator+(Integer other)
{
    Integer result(this->m_value + other.m_value); 
    return result;   //返回结果的对象
}

Integer::~Integer()
{
    cout << "Execute destructor." << endl;
}

int Integer::IntValue()
{
    return m_value;
}