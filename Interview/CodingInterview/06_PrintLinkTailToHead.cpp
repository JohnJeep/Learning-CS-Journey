/*
 * @Author: JohnJeep
 * @Date: 2020-07-26 14:49:05
 * @LastEditTime: 2020-07-26 15:34:51
 * @LastEditors: Please set LastEditors
 * @Description: 题目：输入一链表的头结点，从头到尾打印出链表每个节点的值。
 *               思路：链表从头到尾遍历，输出为从尾到头，需要用到栈的思想。
 * 
 * @FilePath: /06_PrintLinkTailToHead.cpp
 */ 
#include <iostream>
#include <stdlib.h>
#include <stack>
#include "list.h"

using namespace std;



void printListTailToHead(ListNode* head)
{
    stack<ListNode*> node;

    ListNode *p = head;
    while (p != nullptr)
    {
        node.push(p);
        p = p->next;
    }

    while (!node.empty())
    {
        p = node.top();
        printf("%d\n", p->value);
        node.pop();
    }
}

// 测试用例1
void test01()
{
    ListNode *pNode1 = CreateListNode(10);
    ListNode *pNode2 = CreateListNode(20);
    ListNode *pNode3 = CreateListNode(30);
    ListNode *pNode4 = CreateListNode(40);

    ConnectListNodes(pNode1, pNode2);

    printListTailToHead(pNode1);
    DestroyList(pNode1);

}





int main(int argc, char *argv[])
{
    test01();
    return 0;
}