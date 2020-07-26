/*
 * @Author: your name
 * @Date: 2020-07-22 21:18:35
 * @LastEditTime: 2020-07-24 11:24:13
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Journey\DataStructure\LinkList\05_stack.c
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include "01_stack.h"

#define STACK_SIZE       32
int main(int argc, char *argv[])
{
    
    return 0;
}

void stackInit()
{
    sqStack *st;

    st->bottom = (elemType*)malloc(STACK_SIZE * sizeof(elemType));
    if (st->bottom == NULL)
    {
        return;
    }
    st->top = st->bottom;
    st->stackSize = STACK_SIZE;    
}

