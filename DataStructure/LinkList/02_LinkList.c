/*
 * @Author: John
 * @Date: 2019-07-21 21:20:59
 * @Description: 数据结构单链表的实现-----------没有通过
 * @LastEditTime: 2020-07-21 20:37:56
 */
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdbool.h>

typedef struct node
{
    int data;
    struct node *next;
} ListNode, *PtrList;

PtrList createList();
void traverseList(PtrList head);
bool isListEmpty(PtrList head);
PtrList LinkLength(PtrList head);
PtrList getLinkElement(PtrList head, int i);
void insertLink01(PtrList head, int i, int value);
PtrList deleteList(PtrList head, int i);


int main()
{
    PtrList head = NULL; // 链表的头结点为空
    head = (PtrList)malloc(sizeof(PtrList));
    
    head = createList();
    traverseList(head);
    LinkLength(head);
    getLinkElement(head, 3);
    insertLink01(head, 4, 1);
    deleteList(head, 4);

    return 0;
}

/*创建链表
*1. 采用尾插法,在空链表的尾部，将新结点逐个插入到链表的尾部,尾指针p指向链表的尾结点
*2. 初始时，尾结点p指向头结点head，每次读入一个数据元素，则申请一个结点
3. 将新结点插入到尾结点后，p指向新结点
*/ 
PtrList createList()
{
    int len, i, value;  // 结点数len，输入的结点值val
    PtrList head, p, q; // 定义一个头、尾指针、
    head = (PtrList)malloc(sizeof(PtrList));
    head->next = NULL; // 定义空链表
    p = head;          // 结点p指向头结点

    // 判断头结点是否为空
    if (head == NULL)
    {
        printf("程序退出！\n");
        exit(0);
    }
    printf("请输入节点数：len=");
    scanf("%d", &len);

    for (i = 0; i < len; i++)
    {
        q = (PtrList)malloc(sizeof(PtrList));
        printf("输入第%d个结点值：", i + 1);
        scanf("%d", &value);
        if (q == NULL)
        {
            printf("程序退出！\n");
            exit(0);
        }
        q->data = value; // 存入新结点的数据域值
        p->next = q;     // 上一结点的指针域指向新节点q的地址
        p = q;           // 移动尾结点
    }
    q->next = NULL;     // 插入的新结点的指针域为空
    return head;
}

// 遍历链表数据
void traverseList(PtrList head)
{
    PtrList p;
    p = (PtrList)malloc(sizeof(PtrList));
    p = head->next;
    while (p != NULL)
    {
        printf("%d ", p->data);
        p = p->next;
    }
    printf("\n");
}

// 链表的初始化
PtrList initList(PtrList head)
{
    head = (PtrList)malloc(sizeof(PtrList)); // 分配一个空间，头指针head指向头结点
    if (head == NULL)
    {
        exit(1);
    }
    head->next = NULL;
}

// 判断链表是否为空；
// 空链表：头指针和头结点都存在，链表中没有元素
bool isListEmpty(PtrList head)
{
    head = (PtrList)malloc(sizeof(PtrList));
    if (head->next == NULL) // 头指针的指针域是否为空
    {
        /* code */
        return 0;
    }
    else
    {
        return 1;
    }
}

// 销毁单链表，销毁后链表不存在，头结点和头指针都不存在
PtrList destoryList(PtrList head)
{
    head = (PtrList)malloc(sizeof(PtrList));
    PtrList p;
    p = (PtrList)malloc(sizeof(PtrList));

    while (head != NULL) // 最后一个结点的指针域为空
    {
        p = head;          // 新结点p指向头结点
        head = head->next; // 上一个结点的指针域指向下一个结点的地址
        free(p);           // 删除结点p
    }
}

// 清空单链表：链表还存在，链表中的头结点和头指针还存在，只是链表中没有元素
PtrList clearList(PtrList head)
{
    head = (PtrList)malloc(sizeof(PtrList));
    PtrList p, q;
    p = head->next; // 结点p指向第一个结点的地址
    while (p != NULL)
    {
        q = p->next; // 结点q的地址指向p的指针域
        free(p);
        p = q; // 结点p指向结点q
               // q = q->next; //移动并删除q结点，下一个q结点的地址为q->next
    }
    head->next = NULL; // 头指针的指针域置为空
}

// 单链表的表长
PtrList LinkLength(PtrList head)
{
    PtrList p;
    p = (PtrList)malloc(sizeof(PtrList));
    p = head->next;
    int i = 0;
    while (p != NULL)
    {
        i++;
        p = p->next; // p结点的地址指向上一个结点的指针域
    }
    printf("单链表的表长为：%d \n", i);
    return i;
}

/*
从单链表中取第i个元素
只能从链表的头指针开始，链表不是随机存储结构
函数输入：单链表头指针；要查找的结点编号
函数输出：第i个结点的地址
 */
PtrList getLinkElement(PtrList head, int i)
{
    int j = 1; // 当前扫过的结点数，初始值为1
    PtrList p;
    p = (PtrList)malloc(sizeof(PtrList));
    p = head->next;                  // 指针指向第一个结点的地址
    while (p->next != NULL && j < i) // p的指针域为空和查找的元素在链表的总个数范围内
    {
        p = p->next;
        j++;
    }
    if (i == j)
    {
        printf("单链表中第%d个结点的值为：%d \n", i, p->data);
        return p; // 返回第i个结点的地址
    }
    else
    {
        return NULL;
    }
}

/*
链表的插入:在结点a_i之前插入结点
输入：链表头指针，插入点a_i位置，节点值
*/

void insertLink01(PtrList head, int ptr, int value)
{
    PtrList p, s;
    int j = 0;
    s = (PtrList)malloc(sizeof(PtrList));
    //p = (PtrList)malloc(sizeof(PtrList));

    s->data = value;
    p = head;                                // p结点指向头指针
    while ((p->next != NULL) && (j < ptr - 1)) // 从链表的头开始，判断是否为点a_i之前的结点
    {
        p = p->next; // 结点p指向下一个结点的地址，移动结点p
        ++j;
    }
    s->next = p->next; // 新插入结点的指针域指向ptr结点的地址
    p->next = s;       // 新插入结点的地址为a_i的前一个结点的指针域
    printf("插入一个结点后链表为：");
    traverseList(head);
}

// 单链表的删除：删除第i个结点
// 函数输入：链表头指针，结点编号
// 函数输出：被删除结点的地址
PtrList deleteList(PtrList head, int i)
{
    PtrList p, qtr;
    p = head;
    int j =0;
    while ((p->next != NULL) && (j < i -1))
    {
        p = p->next;
        ++j;
    }
    if((p->next == NULL) || (j > i-1)) //p结点的 next域为空或者要找的结点数大于所建链表的长度
        return false;
    qtr = p->next; // 临时保存被删除结点的指针域以备释放空间
    int value = qtr->data; // 临时保存被删除结点的数据域以备释放空间
    p->next = p->next->next;  // p结点的指针域指向p后继结点的后继结点的地址
    //free(qtr);            // 释放要删除节点的空间
    qtr = NULL;
    printf("删除一个结点后链表为：");
    traverseList(head);      // 遍历改变后的链表
}
