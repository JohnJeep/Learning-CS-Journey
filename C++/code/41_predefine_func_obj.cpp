/*
 * @Author: JohnJeep
 * @Date: 2020-07-21 10:27:26
 * @LastEditTime: 2020-07-21 12:03:02
 * @LastEditors: Please set LastEditors
 * @Description: 预定义函数对象
 * @FilePath: /41_predefine_func_obj.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <algorithm>
#include <string>
#include <vector>
#include <set>
#include <functional>

using namespace std;


// 测试用例01
void test01()
{
    cout << "test01 case." << endl;
    plus<int> ps;    // 调用预定于的函数对象
    int x =10;
    int y =20;
    int val = ps(x, y);
    cout << val << endl;

    plus<string> pstr;  // 预定义函数对象字符串相加
    string p1 = "my book ";
    string p2 = "is very beautiful.";
    string p3 = pstr(p1, p2);
    cout << p3 << endl;

    // 使用预定义函数对象sort()排序
    vector<int> vt;
    vector<int>::iterator it;

    vt.push_back(9);
    vt.push_back(5);
    vt.push_back(3);
    vt.push_back(7);
    vt.push_back(1);
    vt.push_back(3);
    sort(vt.begin(), vt.end(), greater<int>());  // 从大到小排序
    for (it = vt.begin(); it != vt.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    // 计算出现的次数, bind2nd() 为函数适配器，绑定equal_to()函数的第二个参数
    // 计算在(begin, end) 迭代器区间内等于 tp 值出现的次数
    int tp = 3;
    int num = count_if(vt.begin(), vt.end(), bind2nd(equal_to<int>(), tp));  
    cout << "num = " << num << endl;
}

class CompareNum
{
private:
    int m_num;
public:
    CompareNum(int num);
    ~CompareNum();
    bool operator() (int n)
    {
        if (n > m_num)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};

CompareNum::CompareNum(int num)
{
    this->m_num = num;
}

CompareNum::~CompareNum()
{
}


void test02()
{
    cout << "test02 case." << endl;
    vector<int> vor;
    vector<int>::iterator it;
    
    for (int i = 0; i <= 10; i++)
    {
        vor.push_back(i + 1);
    }
    for (it = vor.begin(); it != vor.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    // 大于某个数的个数
    int ct1 = count_if(vor.begin(), vor.end(), CompareNum(3)); // 采用谓词的方法实现
    cout << "ct1 = " << ct1 << endl;

    int ct2 = count_if(vor.begin(), vor.end(), bind2nd(greater<int>(), 3)); // 采用预定义的函数对象方法实现
    cout << "ct2 = " << ct2 << endl;

    // 求容器中奇数个数
    int odd = count_if(vor.begin(), vor.end(), bind2nd(modulus<int>(), 2)); // 采用预定义的函数对象方法实现
    cout << "odd = " << odd << endl;

    // 求容器中偶数个数 
    int even = count_if(vor.begin(), vor.end(), not1(bind2nd(modulus<int>(), 2))); // 采用预定义的函数对象方法实现
    cout << "even = " << even << endl;
}


int main(int argc, char *argv[])
{
    test01();
    test02();
    return 0;
}