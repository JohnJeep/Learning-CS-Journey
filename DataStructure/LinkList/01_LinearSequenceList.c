/*
 * @Author: JohnJeep
 * @Date: 2020-07-15 19:57:19
 * @LastEditTime: 2020-07-20 22:23:58
 * @LastEditors: Please set LastEditors
 * @Description: 线性表顺序存储-----------------没有通过
 * @FilePath: /01_LinearSequenceList.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>

#define MAX_SIZE          16
typedef struct tag_sqlist
{
    int length;
    // int data[MAX_SIZE];
    int capacity;
    unsigned int** node;    // 动态分配内存空间
}SqList;

typedef void SqListNode;

typedef struct tag_teacher
{
    int age;
    char *name;
}Teacher;


#if 1
/**
 * @description: 线性表的创建
 * @param {type} 
 * @return: 
 */
SqList* sqListCreate(int size)
{
    SqList* pstr = NULL;
    pstr = (SqList*)malloc(sizeof(SqList));
    if (pstr == NULL)
    {
        printf("pstr error.\n");
        return NULL;
    }

    memset(pstr, 0, sizeof(SqList));
    
    // 根据size空间的大小分配节点空间的大小
    pstr->node = (unsigned int**)malloc(sizeof(unsigned int**) * size);
    if (pstr->node == NULL)
    {   
        printf("pstr->node malloc space error.\n");    
        // return -1;
        return NULL;
    }
    pstr->capacity = size;
    pstr->length = 0;
    return pstr;
}

/**
 * @description: 线性表的销毁: 两次开辟了内存空间，释放内存空间两次
 * @param {type} 
 * @return: 
 */
void sqListDestory(SqList* list)
{
    if (list == NULL)
    {
        return;
    }
    if (list->node != NULL)
    {
        free(list->node);
    }
    
    free(list);
}

/**
 * @description: 清空链表
 * @param {type} 
 * @return: 
 */
void sqListClear(SqList* list)
{
    if (list == NULL)
    {
        return;
    }
    list->length = 0;
}

/**
 * @description: 线性表的长度
 * @param {type} 
 * @return: 
 */
int sqListLength(SqList* list)
{
    if (list == NULL)
    {
        return -1;
    }
    return list->length;   
}

int sqListCapacity(SqList* list)
{
    if (list == NULL)
    {
        return -1;
    }
    return list->capacity;
} 

/**
 * @description: 线性表的插入
 * @param {type} 
 * @return: 
 */
int sqListInsert(SqList *list, SqListNode* node, int pos)
{
    int i;

    if (list == NULL || node == NULL || pos < 0)
    {
        return -1;
    }

    if (list->length >= list->capacity)
    {
        printf("capacity is filled.\n");
        return -1;
    }

    // 容错修正
    if (pos > list->length)
    {
        pos = list->length;
    }
     
    for (i = list->length; i > pos; i--) // 将要插入位置的元素向后移动一位
    {
        list->node[i] = list->node[i-1];
    }
    list->node[i] = node;// 向节点处插入元素

    list->length++;
    return 0;
}

/**
 * @description: 获取指定元素的位置
 * @param {type} 
 * @return: 
 */
SqList* sqListGetLen(SqList* list, int pos)
{
    if (list == NULL || pos < 0)
    {
        return NULL;
    }
    
    return (SqList*)list->node[pos];
}

/**
 * @description: 线性表的删除
 * @param {type} 
 * @return: 
 */
int sqListDelete(SqList* list, int pos)
{
    int i, ret;
    if (list == NULL || pos < 0)
    {
        return -1;
    }

    // *ret = list->node[pos];
    for ( i = pos + 1; i < list->length; i++)
    {
        list->node[i] = list->node[i+1];
    }
    list->length--;
    return 0;
}
 


#endif 


// 删除
// 求长度

int main(int argc, char *argv[])
{
    SqList* p = NULL;
    int ret;
    
    Teacher t1, t2, t3;
    t1.age = 20;
    t2.age = 21;
    t3.age = 22;
    p = sqListCreate(16); 
    ret = sqListInsert(p, (SqListNode*)&t1, 0);
    ret = sqListInsert(p, (SqListNode*)&t2, 0);
    ret = sqListInsert(p, (SqListNode*)&t3, 0);

    for (int i = 0; i < sqListLength(p); i++)
    {
        Teacher* te = (Teacher*)sqListGetLen(p, i);
        if (te != NULL)
        {
            printf("te->age:%d ", te->age);
        }
        printf("\n");
        
    }

    return 0;
}