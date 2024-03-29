/*
 * @Author: JohnJeep
 * @Date: 2019-07-28 14:49:47
 * @LastEditTime: 2020-07-21 20:51:27
 * @Description: 循环链表实现：；两个循环链表相连接
 * @LastEditors: Please set LastEditors
 */
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

typedef struct node{
    int data;
    struct node *next;
}ListLode;

ListLode* creatList(int a[], int n);
void linkLoopList(ListLode* Ta, ListLode* Tb);
void traverseList(ListLode* head);


int main()
{
    int N = 5;
    int a[] = {3, 6, 7, 5, 8};
    int b[] = {2, 4, 9, 1, 10};
    ListLode *Ta, *Tb, *head;
    //head = (PtrNode)malloc(sizeof(PtrNode));

    Ta = creatList(a, N);
    Tb = creatList(b, N);
    head = Ta->next;
    linkLoopList(Ta, Tb);
    //traverseList(head);

    return 0;
}

// 创建单循环链表
// 函数输入：链表结点数组，结点数
// 函数输出：循环链表尾结点q地址
ListLode* creatList(int a[], int n)
{
    ListLode *p, *q, *head;
    head = (ListLode*)malloc(sizeof(ListLode*));
    p = head;
    for(int i = 0; i < n; i++){
        q = (ListLode*)malloc(sizeof(ListLode*));
        q->data = a[i];
        p->next = q;
        p = q;       // 移动指针p,指向下一个结点
    }
    q->next = head;  // 尾节点的指针域指向头结点
    
    return q;
}

// 创建两个循环单链表连接成一个循环链表
// 函数输入：Ta循环单链表尾指针，Tb循环单链表尾指针
// 函数输出：无
void linkLoopList(ListLode* Ta, ListLode* Tb)
{
    ListLode *p;

    p = Ta->next;    // Ta链表的尾节点指向头指针
    Ta->next = Tb->next->next;  // Ta链表尾节点指针域指向Tb链表头节点的地址
    // free(Tb->next);   // 释放Tb链表的头结点地址
    Tb->next = p;     // Tb链表尾节点地址指向p结点
}

// 遍历链表数据
void traverseList(ListLode* head)
{
    ListLode* p;

    p = (ListLode*)malloc(sizeof(ListLode));
    p = head->next;
    while (p != head)
    {
        printf("%d ", p->data);
        p = p->next;
    }
    printf("\n");
}
