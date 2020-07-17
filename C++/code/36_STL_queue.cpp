/*
 * @Author: JohnJeep
 * @Date: 2020-07-17 10:34:43
 * @LastEditTime: 2020-07-17 16:11:23
 * @LastEditors: Please set LastEditors
 * @Description: STL标准库中queue容器
 * @FilePath: /36_STL_queue.cpp
 */ 
#include <iostream>
#include <cstdio>
#include <queue>

using namespace std;

// 普通队列测试用例
void test01()
{
    queue<int> que;
    que.push(11);
    que.push(12);
    que.push(13);
    que.push(14);

    cout << "queue size: " << que.size() << endl;
    cout << "queue front: " << que.front() << endl;
    while (!que.empty())
    {
        int tmp = que.front();
        cout << tmp << " ";
        que.pop();
    } 
    cout << endl;   
}

// 优先级队列测试用例
void test02()
{   
    priority_queue<int> g_priq;                            // 默认为最大值优先队列
    priority_queue<int, vector<int>, greater<int>> l_priq; // 最小值优先队列

    g_priq.push(103);
    g_priq.push(99);
    g_priq.push(107);
    g_priq.push(95);
    size_t ret = g_priq.size();
    cout << "size=" << ret << endl;
    while (!g_priq.empty())   // 出队列
    {
        cout << "max priority queue top: " << g_priq.top() << endl;
        g_priq.pop();
    }
    cout << "pop queue after, size=" << g_priq.size() << endl;
    
    cout << endl;
    l_priq.push(22);
    l_priq.push(11);
    l_priq.push(66);
    l_priq.push(33);
    while (!l_priq.empty())   // 出队列
    {
        cout << "min priority queue top: " << l_priq.top() << endl;
        l_priq.pop();
    }
    cout << "pop queue after, size=" << l_priq.size() << endl; 
}

int main(int argc, char *argv[])
{
    test01();
    test02();
    return 0;
}