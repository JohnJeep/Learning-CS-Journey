/*
 * @Author: JohnJeep
 * @Date: 2020-07-02 19:33:22
 * @LastEditTime: 2020-07-02 21:59:07
 * @LastEditors: Please set LastEditors
 * @Description: 面试题9：用两个栈实现队列
 *               题目：用两个栈实现一个队列。队列的声明如下，请实现它的两个函数appendTail
 *               和deleteHead，分别完成在队列尾部插入结点和在队列头部删除结点的功能。
 * @FilePath: /Interview/剑指offer/01_QueueWithTwoStacks.cpp
 */ 
#include <iostream>
#include <stack>
#include <exception>

using namespace std;

template <typename T> class QueueStack
{
private:
    stack<T> stack1;
    stack<T> stack2;
public:
    QueueStack();
    ~QueueStack();
    void appendTail(const T& element);  // 队尾插入
    T deleteHead();                     // 队头删除
};

template <typename T>
QueueStack<T>::QueueStack()
{
}

template <typename T>
QueueStack<T>::~QueueStack()
{
}

template <typename T>
void QueueStack<T>::appendTail(const T& element)
{
    stack1.push(element);
}

template <typename T>
T QueueStack<T>::deleteHead()
{
    if (stack2.size() <= 0)   // stack2为空时，将stack1中的元素逐个弹出并压入stack2
    {
        while (stack1.size() > 0)
        {
            T& data = stack1.top();
            stack1.pop();
            stack2.push(data);
        }      
    }
    if (stack2.size() == 0)
    {
        // throw new exception("queue is empty");
        cout << "queue is empty" << endl;
    }

    // 弹出stack2栈顶的元素
    T head = stack2.top();
    stack2.pop();

    return head;
}
  
int main()
{
    QueueStack<char> queue;

    queue.appendTail('a');
    queue.appendTail('b');
    queue.appendTail('c');

    char head = queue.deleteHead();
    cout << "head:" << head << endl;

    queue.appendTail('d');
    queue.appendTail('e');
    
    head = queue.deleteHead();
    cout << "head:" << head << endl;
    
    head = queue.deleteHead();
    cout << "head:" << head << endl;

    head = queue.deleteHead();
    cout << "head:" << head << endl;

    return 0;
}
