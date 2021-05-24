/*
 * @Author: JohnJeep
 * @Date: 2020-07-17 11:40:43
 * @LastEditTime: 2021-05-24 14:44:50
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库list容器
 */ 
#include <iostream>
#include <cstdio>
#include <list>
#include <vector>

using namespace std;

void test01()
{
    list<int> st;
    list<int>::iterator index;
    st.push_back(77);
    st.push_back(88);
    st.push_back(99);
    st.push_back(100);
    for (list<int>::iterator it = st.begin(); it != st.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
    
    index = st.begin();
    index++;
    st.insert(index, 200);     // list 不支持 st.begin() + 2 操作，只支持 ++ 操作
   for (list<int>::iterator it = st.begin(); it != st.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    st.insert(st.end(), 999);
    st.insert(st.end(), 999);
    st.insert(st.end(), 999);
    st.insert(st.end(), 999);
    for (list<int>::iterator it = st.begin(); it != st.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;

    st.remove(999);
    for (list<int>::iterator it = st.begin(); it != st.end(); it++)
    {
        cout << *it << " ";
    }
    cout << endl;
}

// list element pass into vector 
void test02() 
{
    cout << "Test02..." << endl;
    list<int> lt = {1, 3, 4, 5, 7, 11, 13, 21};
    vector<int> v(lt.begin(), lt.end());  //将list中的元素依次传入到vector中 

    for (const auto& iter : v) {
        cout << iter << " ";
    }

}
int main(int argc, char *argv[])
{
    // test01();    
    test02();    

    return 0;
}