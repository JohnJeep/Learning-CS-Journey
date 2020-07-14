/*
 * @Author: JohnJeep
 * @Date: 2019-07-29 21:10:25
 * @LastEditTime: 2020-07-14 21:53:34
 * @Description: 实现双向链表：双向链表的插入、双向链表的删除
 * @LastEditors: Please set LastEditors
 */
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

typedef struct doubleNode{
    int data;
    struct doubleNode *prior, *next;
}dLinkList, *DLinkNode;

void insertDLink(dLinkList *p, int x);
DLinkNode getDLink(dLinkList *head, int i);
DLinkNode delteDLink(dLinkList *head, int i);

/**
 * @description: 
 * 1. 建立双向链表，并删除其中的结点：头指针数据域置-1，后面有6个结点
 * 2. 删除指定顺序的结点i=3
 * 3. 删除指定顺序的结点ptr
 * 
 * @param {type} 
 * @return: 
 */
int main()
{
    printf("%d \n", __LINE__);
    printf("%d \n", __TIME__);

    dLinkList *head, *p, *ptr;  // 中间结点p, 要删除结点ptr
    dLinkList x;
    p = (DLinkNode)malloc(sizeof(dLinkList));
    // p = &x;
    p ->data = 6;
    p->prior = NULL;
    p->next = NULL;

    insertDLink(p, -1);
    head = p->prior;     // 指向头结点
    insertDLink(p, 2);
    insertDLink(p, 4);
    insertDLink(p, 6);
    insertDLink(p, 8);
    ptr = p->prior;
    insertDLink(p, 10);

    delteDLink(head, 3);

    return 0;
}

/**
 * @description:在双链表地址为P的结点前插入一个值为X的新结点 
 * @param: 结点地址，插入结点的值
 * @return: 无
 */
void insertDLink(dLinkList *p, int x)
{
    dLinkList *s;
    s = (DLinkNode)malloc(sizeof(dLinkList));
    s->data = x;
    
    s->prior = p->prior;
    p->prior->next = s;
    s->next = p;
    p->prior = s;
}

/**
 * @description:找到第i个结点的地址，跳过头结点 
 * @param: 双链表首地址，带查找的结点编号i
 * @return: 第i个结点的地址
 */
DLinkNode getDLink(dLinkList *head, int i)
{
    int j = 1;
    dLinkList *ptr;
    ptr = head->next;
    while ((j < i) && (ptr->next != NULL))
    {
        ptr = ptr->next;
    }
    printf("%d", *ptr);
    return ptr;
}

/**
 * @description: 在双链表中删除第i个结点
 * @param: 双向链表首地址，待删除结点编号
 * @return: 双向链表首地址
 */
DLinkNode delteDLink(dLinkList *head, int i)
{
    dLinkList *p;
    p = (DLinkNode)malloc(sizeof(DLinkNode));
    p = getDLink(head, i);
    if (p != NULL)
    {
        p->prior->next = p->next;   // 要删除结点的前一个结点的next域指向要删除结点next域的地址
        p->next->prior = p->prior;  // 要删除结点next域的地址的prior域地址指向要删除结点prior域的地址
    }
    return p;
}