/*
 * @Author: JohnJeep
 * @Date: 2020-08-20 08:53:29
 * @LastEditTime: 2020-08-20 10:45:37
 * @LastEditors: Please set LastEditors
 * @Description: 题目：合并连个排序的链表
 *               描述：输入两个单调递增的链表，输出两个链表合成后的链表，当然我们需要合成后的链表满足单调不减规则。
 *               思路：
 *                    法一：递归方法实现
 * 
 *                    法二：迭代方法
 *                          1、开始时，若链表A指向的结点值（头结点）小于链表B指向的结点值，则将cur指针指向链表A指向的结点，
 *                          2、再依次移动链表A的结点。否则就是将cur指针指向链表B指向的结点，再依次移动链表B的结点。
 *                          3、当链表A后链表B的尾结点为nullptr时，结束循环。
 *                          4、最后将pHead1或pHead2指针改变指向后，再将cur指向pHead1或pHead2指针
 * 
 * 
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

/**
 * @description: 法一：递归方法实现
 * @param {type} 
 * @return {type} 
 */
ListNode* mergeTwoListRecur(ListNode* pHead1, ListNode* pHead2)
{
    // 任意一个链表为空时，合并后的链表是非空的那个链表
    if (pHead1 == nullptr)
    {
        return pHead2;
    }
    if (pHead2 == nullptr)
    {
        return pHead1;
    }

    ListNode* pNewHead = nullptr;

    if (pHead1->val < pHead2->val)
    {
        pNewHead = pHead1;
        pNewHead->next = mergeTwoListRecur(pHead2, pHead1->next);
    }
    else
    {
        pNewHead = pHead2;
        pNewHead->next = mergeTwoListRecur(pHead1, pHead2->next);
    }        

    return pNewHead;
}

/**
 * @description: 法二：迭代方法
 * @param {type} 
 * @return {type} 
 */
ListNode* mergeTwoListIterm(ListNode* pHead1, ListNode* pHead2)
{
    if (pHead1 == nullptr)
    {
        return pHead2;
    }
    if (pHead2 == nullptr)
    {
        return pHead1;
    }

    ListNode *vHead = new ListNode(-1);  // 建立一个虚拟的头结点，后面每个结点都有一个前驱结点
    ListNode *cur = vHead;
    while (pHead1 && pHead2) 
    {
        if (pHead1->val < pHead2->val) 
        {
            cur->next = pHead1;     // 改变cur指针的指向
            pHead1 = pHead1->next;  // 移动pHead1指针
        }
        else 
        {
            cur->next = pHead2;
            pHead2 = pHead2->next;
        }
        cur = cur->next;      // 移动当前结点的指针
    }
    cur->next = pHead1 ? pHead1 : pHead2;  // pHead1或pHead2指针改变指向后，再将cur指向pHead1或pHead2指针
    return vHead->next;
}

int main(int argc, char *argv[])
{
    
    return 0;
}