/*
 * @Author: JohnJeep
 * @Date: 2020-07-15 19:57:19
 * @LastEditTime: 2020-07-15 23:26:15
 * @LastEditors: Please set LastEditors
 * @Description: 线性表顺序存储
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
    int data[MAX_SIZE];
    // int capacity;
    // unsigned int** node;    // 动态分配内存空间
}SqList;



#if 0
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
        return -1;
    }

    memset(pstr, 0, sizeof(SqList));
    
    // 根据size空间的大小分配节点空间的大小
    pstr->node = (unsigned int*)malloc(sizeof(unsigned int*) * size);
    if (pstr->node == NULL)
    {   
        printf("pstr->node malloc space error.\n");    
        return -1;
    }
    pstr->capacity = size;
    pstr->length = 0;
    return pstr;
    
}

/**
 * @description: 线性表的销毁
 * @param {type} 
 * @return: 
 */
void sqListDestory(SqList* list)
{
    SqList* p = NULL;
    if (list == NULL)
    {
        return;
    }
    if (p->node != NULL)
    {
        free(p->node);
    }
    
    free(p);
}

void sqListClear(SqList* list)
{
    if (list != NULL)
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
    if (list != NULL)
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
SqList* sqListInsert(SqList *list)
{

}
#endif 


// 删除
// 求长度

int main(int argc, char *argv[])
{
    
    return 0;
}