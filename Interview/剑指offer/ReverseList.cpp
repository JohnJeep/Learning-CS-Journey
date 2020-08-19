/*
 * @Author: JohnJeep
 * @Date: 2020-08-19 11:57:14
 * @LastEditTime: 2020-08-19 14:35:00
 * @LastEditors: Please set LastEditors
 * @Description: 题目：反转链表
 *               描述：输入一个链表，反转链表后，输出新链表的表头。
 *               思路：初始化三个指针
 *                     rPre为反转链表后的一个结点指针，开始时没有反转，因此为nullptr，
 *                     PCurrent为将要反转链表的当前结点指针，最开始第一个结点需要反转，所以指向head
 *                     pnext为将要反转链表的下一个结点指针，目的是保存链表，因为PCurrent改变指向后，后面的链表失效了。
 * 
 *                     pNext = pCurrent->next;     // 保存当前的指针
 *                     pCurrent->next = rPre;      // 改变pCurrent->next指针的指向，需要反转链表的当前结点的下个指针指向反转后链表的最后一个结点
 *                     rPre = pCurrent;            // 移动rPre指针，将rPre指针指向当前指针
 *                     pCurrent = pNext;           // 移动pCurrent指针，将pCurrent指针指向下一个结点的指针
 * 
 *               关键点：先改变指针指向，再移动指针
 *               复杂度：时间复杂度：O(n), 遍历一次链表；空间复杂度 O(1)
 */
#include <iostream>
#include <cstdio>

using namespace std;

struct ListNode {
	int val;
	struct ListNode *next;
	ListNode(int x) :
			val(x), next(NULL) {
	}
};

ListNode* ReverseList(ListNode* pHead) 
{
    ListNode* rPre = nullptr;      // 反转链表后的一个结点指针，开始时没有反转，因此为nullptr
    ListNode* pCurrent = pHead;    // 将要反转链表的当前结点指针
    ListNode* pNext = nullptr;     // 将要反转链表的下一个结点指针
    
    while(pCurrent)
    {
        pNext = pCurrent->next;     // 保存当前的指针
        pCurrent->next = rPre;      // 需要反转链表的当前结点的下个指针指向反转后链表的最后一个结点
        rPre = pCurrent;            // 移动rPre指针，将rPre指针指向当前指针
        pCurrent = pNext;           // 移动pCurrent指针，将pCurrent指针指向下一个结点的指针
    }

    return rPre;
}
int main(int argc, char *argv[])
{
    
    return 0;
}