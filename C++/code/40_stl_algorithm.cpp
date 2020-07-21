/*
 * @Author: JohnJeep
 * @Date: 2020-07-20 10:41:42
 * @LastEditTime: 2020-07-21 15:51:10
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /40_stl_algorithm.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <iterator>

using namespace std;

void showVector(vector<int>& vct)
{
    vector<int>::iterator it;
    for (it = vct.begin(); it != vct.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;    
}

// 测试用例01
template <typename T>
class AddFunc
{
private:
    /* data */
public:
    AddFunc(/* args */);
    ~AddFunc();

    // 两个函数参数的二元谓词运算
    T operator() (T left, T right)   // 重写 () 操作运算符，参数类型传递的是 value
    {
        return left + right;
    }
};

template <typename T>
AddFunc<T>::AddFunc(/* args */)
{
}

template <typename T>
AddFunc<T>::~AddFunc()
{
}

void test01()
{
    cout << "test01 case!" << endl;
    vector<int> v1, v2, v3;

    for (int i = 0; i < 5; i++)
    {
        v1.push_back(i + 1);
    }
    for (int j = 5; j < 10; j++)
    {
        v2.push_back(j + 1);
    } 

    cout << "v1 element: ";
    showVector(v1);
    
    cout << "v2 element: ";
    showVector(v2);

    // find_if();
    v3.resize(5);    // 两个容器相加之前需要给相加结果的容器分配大小，否则会报错
    showVector(v3);

    transform(v1.begin(), v1.end(), v2.begin(), v3.begin(), AddFunc<int>());   // addFunc<int>()是匿名函数对象
    cout << "v3 element: ";
    showVector(v3);

    // 直接将transform()转换后的函数输出到终端上
    cout << "transform() output terminal: ";
    transform(v1.begin(), v1.end(), ostream_iterator<int>(cout, " "), negate<int>());   // 使用标准库中预定义的函数对象
    cout << endl;
}

// 测试用例02
void traverseEle(int& n)  // for_each()底层调用回调函数： __f(*__first);
{
    cout << n << " ";
}

template<typename T>
class TravEle
{
private:
    /* data */
public:
    TravEle(/* args */);
    ~TravEle();
    void operator() (T& num)   // 参数类型传递的是 reference
    {
        cout << num << " ";
    }
};

template<typename T>
TravEle<T>::TravEle(/* args */)
{
}

template<typename T>
TravEle<T>::~TravEle()
{
}

// for_each();  // 遍历容器中的所有元素
void test02()
{
    cout << endl;
    cout << "test02 case!" << endl;
    vector<int> vor;
    for (int i = 0; i < 8; i++)
    {
        vor.push_back(i + 1);
    }
    cout << "iterator traverse element: ";
    showVector(vor);

    cout << "for_earch() traverse element: ";
    for_each(vor.begin(), vor.end(), traverseEle);  // 回调函数方式实现，传递的是函数的首地址
    cout << endl;

    cout << "function object traverse element: ";
    for_each(vor.begin(), vor.end(), TravEle<int>());  // 通过函数对象实现
}

int main(int argc, char *argv[])
{
    test01();
    test02();
    
    return 0;
}