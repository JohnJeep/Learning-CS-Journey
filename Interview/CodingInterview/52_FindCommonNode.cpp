/*
 * @Author: JohnJeep
 * @Date: 2020-08-18 23:19:13
 * @LastEditTime: 2020-08-18 23:49:01
 * @LastEditors: Please set LastEditors
 * @Description: 题目：两个链表的公共结点
 *               描述：输入两个链表，找出它们的第一个公共结点。
 * 
 *               思路：1、首先遍历两个链表分别得到它们的长度，知道哪个链表比较长，以及长的链表比短的链表多几个结点？
 *                     2、第二次遍历时，先在较长的链表上多走若干步，然后同时在两个链表上遍历。
 *                     3、遍历找到的第一个相同结点就是它们的第一个公共结点。
 * 
 */
#include <iostream>
#include <stdlib.h>

using namespace std;

struct ListNode {
	int val;
	struct ListNode *next;
	ListNode(int x) :
			val(x), next(NULL) {
	}
};

int getListLength(ListNode* p)
{
    int length = 0;
    ListNode* pNode = p;
    while (pNode !=nullptr)
    {
        ++length;
        pNode = pNode->next;
    }
    
    return length;
}

ListNode* findFirstCommonNode(ListNode* pHead1, ListNode* pHead2)
{
    int len1 = getListLength(pHead1);
    int len2 = getListLength(pHead2);
    int delta = len1 - len2;
    ListNode* pListNodeLong = pHead1;
    ListNode* pListNodeShort = pHead2;

    if (len2 > len1)
    {
        pListNodeLong = pHead2;
        pListNodeShort = pHead1;
        delta = len2 - len1;        
    }

    // 现在长链表上走几步，再同时遍历两个链表
    for (int i = 0; i < delta; i++)
    {
        pListNodeLong = pListNodeLong->next;
    }
    while ((pListNodeLong != nullptr) && (pListNodeShort != nullptr) && (pListNodeLong != pListNodeShort))
    {
        pListNodeLong = pListNodeLong->next;
        pListNodeShort = pListNodeShort->next;
    }

    // 得到第一个公共结点
    ListNode* pFirstCommonNode = pListNodeLong;

    return pFirstCommonNode;
}


int main(int argc, char *argv[])
{
    
    return 0;
}