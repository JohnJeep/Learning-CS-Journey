/*
 * @Author: JohnJeep
 * @Date: 2020-07-16 22:03:51
 * @LastEditTime: 2020-08-20 11:54:56
 * @LastEditors: Please set LastEditors
 * @Description: 线性表的链式实现函数主体
 * @FilePath: /02_LinearChainList.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "02_ChainList.h"

// 声明链表的业务节点
typedef struct tag_Teacher
{
    LinKListNode node;
    int age;
    char name[16];
}Teacher;


int main(int argc, char *argv[])
{
    int len, ret, i;
    LinkList *list = NULL;
    Teacher t1, t2, t3, t4;
    t1.age = 23;
    t2.age = 25;
    t3.age = 24;
    t4.age = 20;

    list = linkListCreate();
    if (list == NULL)
    {
        return -1;
    }
    len = linkListLength(list);
    printf("insert before link list length: %d\n", len);

    ret = linkListInsert(list, (LinKListNode*) &t1, 0);
    ret = linkListInsert(list, (LinKListNode*) &t2, 0);
    ret = linkListInsert(list, (LinKListNode*) &t3, 0);
    ret = linkListInsert(list, (LinKListNode*) &t4, 0);
    printf("ret = %d\n", ret);
    len = linkListLength(list);
    printf("insert after link list length: %d\n", len);

    // 获取链表中的节点
    printf("遍历链表中的节点: ");
    for (i = 0; i < linkListLength(list); i++)
    {
        Teacher* temp = (Teacher*)linkListGet(list, i);
        if (temp == NULL)
        {
            return -1;
        }
        printf("temp->age:%d\t", temp->age);
    }

    // 删除链表
    printf("\n删除链表中的节点: ");
    while (linkListLength(list) > 0)
    {
        Teacher* del = (Teacher*)linkListDelete(list, 0);
        if (del == NULL)
        {
            return -1;
        }
        printf("del->age:%d \t", del->age);
    }
    printf("\n");
    
    len = linkListLength(list);
    printf("delete after link list length: %d\n", len);
    
    return 0;
}

/**
 * @description: // 创建链表
 * @param {type} 
 * @return: 
 */
LinkList* linkListCreate()
{
    S_LinkList* list = NULL;
    list  = (S_LinkList*)malloc(sizeof(S_LinkList));
    if (list == NULL)
    {
        return NULL;
    }
    memset(list, 0, sizeof(S_LinkList));
    return list;
} 

/**
 * @description: 销毁链表
 * @param {type} 
 * @return: 
 */
void linkListDestory(LinkList* list)
{
    if (list != NULL)
    {
        free(list);
        list = NULL;
    }
}                           

/**
 * @description: 链表恢复到初始状态
 * @param {type} 
 * @return: 
 */
void linkListClear(LinkList* list)
{
    S_LinkList* pList = NULL;
    if (list != NULL)
    {
        pList = (S_LinkList*)list;
        pList->length = 0;
        pList->header.next = NULL;
    }
}                               

/**
 * @description: 获得链表的长度
 * @param {type} 
 * @return: 
 */
int linkListLength(LinkList* list)
{
    S_LinkList* pList = NULL;
    if (list != NULL)
    {
        pList = (S_LinkList*)list;
        return pList->length;
    }    
    else
    {
        return -1;
    }
}       

/**
 * @description: 向链表中插入节点：两个关键的中间指针变量
 *               1、将需要插入的结点指向插入结点的后一个结点位置
 *               2、将需要插入的结点的前一个结点指向当前插入的结点位置
 * @param {type} 
 * @return: 
 */
int linkListInsert(LinkList* list, LinKListNode* node, int pos)
{
    LinKListNode* current = NULL;  // 为新插入的节点
    S_LinkList* pList = NULL;

    if (list == NULL || node == NULL || pos < 0)
    {
        return -1;
    }
    
    pList = (S_LinkList*)list;
    current = &(pList->header);    // 指向头结点的地址
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;
    }
    node->next = current->next;  // 第一步
    current->next = node;        // 第二步
    pList->length++;             // 每次插入一个节点，链表长度加一

    return 0;
}

/**
 * @description: 获得链表中的元素
 * @param {type} 
 * @return: 
 */
LinKListNode* linkListGet(LinkList* list, int pos)
{
    if (list == NULL || pos < 0)
    {
        return NULL;
    }    LinKListNode* current = NULL;  // 为新插入的节点

    S_LinkList* pList = NULL;

    pList = (S_LinkList*)list;
    current = &(pList->header);    // 辅助指针变量指向头结点的地址
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;
    }
    return current->next;
} 

/**
 * @description: 链表节点的删除
 *               1、删除前需要先保存要删除的结点
 *               2、将当前需要删除结点的前一个结点指向当前需要删除结点的后一个结点
 *                
 * @param {type} 
 * @return: 
 */
LinKListNode* linkListDelete(LinkList* list, int pos)
{
    LinKListNode* current = NULL;  // 为新插入的节点
    S_LinkList* pList = NULL;
    LinKListNode* cache;

    if (list == NULL || pos < 0)
    {
        return NULL;
    }
    
    pList = (S_LinkList*)list;
    current = &(pList->header);    // 指向头结点的地址
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;
    } 

    cache = current->next;           // 关键步骤一
    current->next = cache->next;     // 关键步骤二
    pList->length--;

    return cache;
}