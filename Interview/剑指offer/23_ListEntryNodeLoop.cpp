/*
 * @Author: JohnJeep
 * @Date: 2020-08-19 14:37:21
 * @LastEditTime: 2020-08-19 16:16:51
 * @LastEditors: Please set LastEditors
 * @Description: 题目：链表中环的入口结点
 *               描述：给定一个单链表，如果有环，返回环的入口结点，否则，返回nullptr
 *               思路：
 *                  法一：哈希法
 *                        1、遍历链表的每个结点
 *                        2、如果当前结点的地址没有出现在set中，则存入set；
 *                        3、若当前结点的地址出现在set中，则当前结点就是环的入口结点
 *                        4、整个链表遍历完，若当前结点的地址没有出现在set中，则不存在环
 *                  复杂度：时间复杂度 O(n)    空间复杂度 O(n)，最坏情况下，单链表的所有结点都在存入set
 *                 
 *                 法二：双指针法
 *                        1、快指针和慢指针都指向头结点，让快指针每次移动两步，慢指针每次移动一步，第一次相遇时，就停止。
 *                        2、再次重新将快指针指向头结点，慢指针指向不变（在环内），让慢指针和快指针每次移动一步，
 *                        3、当两个指针再次相遇时，相遇的结点就是环的入口结点。
 *                 复杂度：时间复杂度 O(n)    空间复杂度 O(1)
 * 
 *                 法三：
 *                        1、判断链表中是否有环
 *                        2、得到环中结点的数
 *                        3、找到环的入口结点
 */
#include <iostream>
#include <cstdio>
#include <unordered_set>

using namespace std;

struct ListNode {
    int val;
    struct ListNode *next;
    ListNode(int x) :
        val(x), next(NULL) {
    }
};

// 法一
ListNode* EntryNodeOfLoopSet(ListNode* pHead)
{
    if (pHead == nullptr)
    {
        return nullptr;
    }
    unordered_set<ListNode*> ust;
    while (pHead)
    {
        if (ust.find(pHead) == ust.end())
        {
            ust.insert(pHead);
            pHead = pHead->next;
        }
        else
        {
            return pHead;
        }
    }
    
    return nullptr;
}

// 法二
ListNode* EntryNodeOfLoopFastSlow(ListNode* pHead)
{
    ListNode* fast = pHead;
    ListNode* slow = pHead;
    if (fast == nullptr || fast->next == nullptr)
    {
        return nullptr;
    }
    while (fast && fast->next)     // fast指针和fast的下一个指针都不为空
    {
        fast = fast->next->next;   // fast指针移动两步
        slow = slow->next;         // slow指针移动一步
        if (fast == slow)
        {
            break;
        }
    }

    fast = pHead;
    while (fast != slow)
    {
        fast = fast->next;
        slow = slow->next;
    }
    
    return fast;
}
int main(int argc, char *argv[])
{
    
    return 0;
}
