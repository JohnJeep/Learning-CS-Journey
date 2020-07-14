/*
 * @Author: JohnJeep
 * @Date: 2019-07-18 22:30:28
 * @LastEditTime: 2020-07-14 21:45:54
 * @Description: 简单实现单链表的增删、改、查
 */
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

// 定义结点结构体
typedef struct node{
    int data; // 数据域
    struct node *next; //下一个结点

}ListNode, *PList;

ListNode *createLinkList(int a[], int n)    // 指针函数，返回值为一个地址
{
    // 单链表尾部插入结点
    ListNode *head, *p, *q;
    int i;
    head = (ListNode *)malloc(sizeof(ListNode));  // 给头指针动态分配一个内存
    q = head;
    for(i = 0; i < n; i++){
        p = (ListNode *)malloc(sizeof(ListNode));   // 给指针P动态分配内存
        p->data = a[i];   // 向p结点中添加值
        q->next = p;      // 把p的地址添加到前驱q的指针域中
        p = q;
    }
    p->next = NULL;   // 尾节点指针域置NULL 
    return head;
}

int main()
{
    int len;
    int a[] = {2, 4, 6, 8, 10};
    printf("请输入结点值：");
    scanf("%d", &len);
    for (size_t i = 0; i < len; i++)
    {

        /* code */
    }
    
    createLinkList(a, 5);

    return 0;
}


