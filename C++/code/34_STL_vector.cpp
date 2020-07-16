/*
 * @Author: JohnJeep
 * @Date: 2020-07-16 10:48:40
 * @LastEditTime: 2020-07-16 16:17:12
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库中vector常见用法
 * @FilePath: /34_STL_vector.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <vector>

using namespace std;

void test01()
{
    // vector首尾元素的添加、获取、删除
    vector<int> vct1;
    cout << "push element before, vct1 size: " << vct1.size() << endl;
    vct1.push_back(1);
    vct1.push_back(3);
    vct1.push_back(5);
    vct1.push_back(7);
    cout << "push element after, vct1 size: " << vct1.size() << endl;

    vct1.pop_back();
    for (size_t i = 0; i < vct1.size(); i++)
    {
        cout << "vector element: " << vct1[i] << endl;  // 通过数组的方式访问vector
    }
    cout << endl;

    // 首尾元素的修改
    vct1.front() = 10;
    vct1.back() = 20;

    for (size_t j = 0; j < vct1.size(); j++)
    {
        cout << "amend vector element aafter: " << vct1[j] << endl;
    }
    cout << endl;
}

// 通过迭代器的方式遍历数组
void test02()
{
    vector<int> vct2(10);   // 给vector分配 10 个 int 类型的内存空间
    for (int i = 0; i < 10; i++)
    {
        vct2[i] = i + 1;
    }

    // 正向遍历
    cout << "迭代器正向遍历: ";
    for (vector<int>::iterator it = vct2.begin(); it != vct2.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    // 反向遍历
    cout << "迭代器反向遍历: ";
    for (vector<int>::reverse_iterator rit = vct2.rbegin(); rit != vct2.rend(); rit++)
    {
        cout << *rit << " ";
    }
    cout << endl;
}

// vector删除
void test03()
{
    vector<int> vct2(10);   // 给vector分配 10 个 int 类型的内存空间
    cout << "origin vector element: ";
    for (int i = 0; i < 10; i++)
    {
        vct2[i] = i + 1;
        cout << vct2[i] << " ";
    }
    cout << endl;
    
    vct2.erase(vct2.begin()); // 删除指定位置的值
    vct2.erase(vct2.begin() + 3, vct2.begin() + 5);   // 删除指定区间 [3, 5) 的值
    cout << "delete specific region vector element: ";
    for (size_t i = 0; i < vct2.size(); i++)
    {
        cout << vct2[i] << " ";  // 通过数组的方式访问vector
    }
    cout << endl; 

    vct2.push_back(5);
    vct2.push_back(5);
    vct2.push_back(7);
    vct2.push_back(5);
    vct2.push_back(5);
    cout << "vector element: ";
    for (size_t i = 0; i < vct2.size(); i++)
    {
        cout << vct2[i] << " ";  // 通过数组的方式访问vector
    }
    cout << endl;

    // 删除vector中指定的元素值
    for (vector<int>::iterator it = vct2.begin(); it != vct2.end();)
    {
        if (*it == 5)
        {
           it = vct2.erase(it);   // 返回值为下一个元素的首地址
        }
        else
        {
            it++;
        }
    }
    cout << "delete vector element: ";
    for (size_t i = 0; i < vct2.size(); i++)
    {
        cout << vct2[i] << " ";  // 通过数组的方式访问vector
    }
    cout << endl;
}
int main(int argc, char *argv[])
{
    // test01();
    // test02();
    test03();
    return 0;
}
