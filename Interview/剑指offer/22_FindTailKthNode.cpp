/*
 * @Author: JohnJeep
 * @Date: 2020-08-19 10:24:18
 * @LastEditTime: 2020-08-19 12:01:30
 * @LastEditors: Please set LastEditors
 * @Description: 题目：查找单链表中倒数第 k 个结点
 *               描述：输入一个链表，输出该链表中倒数第k个结点。
 * 
 *               思路：采用快慢指针的思想。快慢指针之间相差 k 个指针，从头结点开始，先让快指针走 k 步，
 *                     然后慢指针开始与快指针同步走，直到当快指针指向尾结点时（null），
 *                     慢指针就指向倒数第 k 个结点了。
 *               
 *               时间复杂度：O(n)，只遍历一次链表; 空间复杂度：O(1)
 */
#include <iostream>
#include <cstdio>
#include <stack>

using namespace std;

struct ListNode {
	int val;
	struct ListNode *next;
	ListNode(int x) :
			val(x), next(NULL) {
	}
};


// 不正确
ListNode* findTailKthNode(ListNode* pListHead, int k)
{
    stack<ListNode*> st;

    if (pListHead == nullptr || k <= 0)
    {
        return nullptr;
    }
    while (pListHead)
    {
        st.push(pListHead);
        pListHead = pListHead->next;
    }

    while (k)
    {
        st.pop();
        k--;
    }
    return st.top();
}

ListNode* FindKthToTail(ListNode* pListHead, unsigned int k) 
{
    if (pListHead == nullptr || k <= 0)
    {
        return nullptr;
    }
    
    ListNode* slow = pListHead;
    ListNode* fast = pListHead;
    while (k--)
    {
        if (fast)
        {
            fast = fast->next;
        }
        else
        {
            return nullptr;   // 单链表长度小于 k，则直接返回
        }
    }

    while (fast)
    {
        slow = slow->next;
        fast = fast->next;
    }
    return slow;
}

int main(int argc, char *argv[])
{
    
    return 0;
}
