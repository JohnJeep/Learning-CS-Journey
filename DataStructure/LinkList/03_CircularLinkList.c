/*
 * @Author: JohnJeep
 * @Date: 2020-07-21 21:23:51
 * @LastEditTime: 2020-07-22 20:52:19
 * @LastEditors: Please set LastEditors
 * @Description: 循环链表的实现-------------------没有调通
 * @FilePath: /03_CircularLinkList.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include "03_circularLinkList.h"

typedef struct tag_circularList
{
    int data;
    CircularListNode header;
}circular;


int main(int argc, char *argv[])
{
    // 测试用例：解决约瑟夫问题
    circular c1, c2, c3, c4, c5, c6, c7, c8;

    CircularList *cirList = NULL;
    cirList = CircularListCreate();
    if (cirList == NULL)
    {
        printf("链表创建失败...\n");
        return -1;
    }
    c1.data = 1;
    c2.data = 2;
    c3.data = 3;
    c4.data = 4;
    c5.data = 5;
    c6.data = 6;
    c7.data = 7;
    c8.data = 8;

    CircularListInsert(cirList, (CircularListNode*)&c1, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c2, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c3, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c4, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c5, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c6, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c7, CircularListLength(cirList));
    CircularListInsert(cirList, (CircularListNode*)&c8, CircularListLength(cirList));
    printf("length = %d\n", CircularListLength(cirList));

    for (int i = 0; i < CircularListLength(cirList); i++)
    {
        circular* ret = (circular*)CircularListNext(cirList);
        printf("%d\n", ret->data);
    }

    CircularListReset(cirList);
    while (CircularListLength(cirList) > 0)
    {
        for (int i = 0; i < 3; i++)
        {
            CircularListNext(cirList);
        }
        circular* current = (circular*)CircularListCurrent(cirList);   // 获取游标所指向的位置
        if (current == NULL)
        {
            printf("current pointer error.\n");
        }
        printf("%d \n", current->data);  // 打印当前游标所指向的值
        CircularListDeleteNode(cirList, (CircularListNode*)current);  // 通过节点的值删除节点
        CircularListDelete(cirList, CircularListLength(cirList));
    }
    CircularListDestory(cirList);

    return 0;
}


CircularList* CircularListCreate()
{
    C_CircularList *list = NULL;
    list = (C_CircularList*)malloc(sizeof(C_CircularList));
    if (list == NULL)
    {
        return NULL;
    }
    memset(list, 0 , sizeof(C_CircularList));
    
    return list;
}


void CircularListDestory(CircularList* list)
{
    if (list != NULL)
    {
        free(list); 
        list = NULL;
    }
}               

void CircularListClear(CircularList* list)
{
    C_CircularList *cirList = (C_CircularList*)list;
    if (cirList != NULL)
    {
        cirList->length = 0;
        cirList->header.next = NULL;
    }
}            

int CircularListLength(CircularList* list)
{
    C_CircularList *cirList = (C_CircularList*)list;
    if (cirList == NULL)
    {
        return -1;
    }
    return cirList->length;
}   

/**
 * @description: 循环链表的插入
 * @param {type} list: 链表
 * @param {type} node: 节点
 * @param {type} pos: 插入的位置
 * @return: 
 */
int CircularListInsert(CircularList* list, CircularListNode* node, int pos)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode *current = NULL;

    if (cirList == NULL || node == NULL || pos < 0)
    {
        return -1;
    }
    current = &(cirList->header);    // 当前指针指向头指针
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;     // 移动节点
    }
    node->next = current->next;
    current->next = node;

    // 第一次插入节点
    if (cirList->length == 0)
    {
        cirList->slider = node;     // 游标指向插入的节点
    }
    cirList->length++;

    // 采用头插法向循环链表中插入，current节点指向链表的头部，需要获取最后一个元素
    if (current == (CircularListNode*)cirList)
    {
        CircularListNode* last = CircularListGet(cirList, cirList->length - 1);  // 获取最后一个元素
        last->next = current->next;
    }

    return 0;
}

/**
 * @description: 获得当前循环链表中的指定位置的元素
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListGet(CircularList* list, int pos)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode* current = NULL;

    if (cirList == NULL || pos < 0)
    {
        return NULL;
    }
    // current = &(cirList->header);
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;    // 移动节点
    }
    return current->next;
}

/**
 * @description: 通过节点的位置，删除循环链表中的普通节点和头节点
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListDelete(CircularList* list, int pos)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode *current = NULL;
    CircularListNode *cache = NULL;
    CircularListNode *last = NULL;            // 最后一个节点

    if (cirList == NULL || pos < 0)
    {
        return NULL;
    }
    current = &(cirList->header);
    for (int i = 0; (i < pos) && (current->next != NULL); i++)
    {
        current = current->next;    // 移动节点
    }

    //删除的是头结点
    if (current == (CircularListNode*)cirList)
    {
        last = (CircularListNode*)CircularListGet(cirList, cirList->length - 1);
    }

    // 删除普通节点
    cache = current->next;          // 缓存被删除的节点位置
    current->next = cache;
    cirList->length--;

    // 判断链表知否为空
    if (last != NULL)
    {
        cirList->header.next = cache->next;
        last->next = cache->next;
    }
    
    // 删除的元素为游标所指向的元素
    if (cirList->slider == cache)
    {
        cirList->slider = cache->next;
    }
    
    // 元素删除后，链表长度为0
    if (cirList->length == 0)
    {
        cirList->header.next = NULL;
        cirList->slider = NULL;
    }
    
    return cache;
}

/**
 * @description: 通过节点的值，删除链表中的节点
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListDeleteNode(CircularList* list, CircularListNode* node)
{
    C_CircularList *cirList = (C_CircularList*)list; 
    CircularListNode *ret = NULL;
    int i = 0;

    if ((cirList != NULL) && (node != NULL))
    {
        CircularListNode* current = (CircularListNode*)cirList;
        for (i = 0; i < cirList->length; i++)
        {
            if (current->next == node)
            {
                ret = current->next; 
                break;
            }
            current = current->next;  // 移动指针
        }
    }
    
    if (ret != NULL)
    {
        CircularListDelete(cirList, i);
    }
    return ret;
}

/**
 * @description: 让游标重新指向链表的头部
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListReset(CircularList* list)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode *ret = NULL;

    if (cirList == NULL || ret == NULL)
    {
        return NULL;
    }
    cirList->slider = cirList->header.next;
    ret = cirList->slider;

    return ret;
}

/**
 * @description: 获取游标所指向的位置
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListCurrent(CircularList* list)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode *ret = NULL;

    if (cirList != NULL)
    {
        ret = cirList->slider;    
    }
    return ret;
}  

/**
 * @description: 获取当前游标所指向的位置，并让游标下移
 * @param {type} 
 * @return: 
 */
CircularListNode* CircularListNext(CircularList* list)
{
    C_CircularList *cirList = (C_CircularList*)list;
    CircularListNode *ret = NULL;

    if (cirList == NULL || cirList->slider == NULL)
    {
        return NULL;
    }
    ret = cirList->slider;
    cirList->slider = ret->next;

    return ret;
}                     
