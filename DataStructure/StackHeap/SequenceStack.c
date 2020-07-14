/*
 * @Author: JohnJeep
 * @Date: 2019-07-30 21:08:43
 * @LastEditTime: 2020-07-14 21:58:01
 * @Description: 顺序栈的实现：栈的初始化、判栈为空、取栈顶元素、进栈、出栈
 * @LastEditors: Please set LastEditors
 */
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <stdbool.h>

#define STACK_SIZE 20
typedef int dataType;

typedef struct qStack
{
    dataType stack[STACK_SIZE];            // 栈空间
    int top;                       // 栈顶指针
}SqeStack;

SqeStack *initSqStack();
void stackShow(SqeStack *s);
bool sqStackEmpty(SqeStack *s);
int sqStackLength(SqeStack *s);   
int pushStack(SqeStack *s, dataType value);
void popStack(SqeStack *s);
void stackShow(SqeStack *s);
void clearStack(SqeStack *s);
void freeStack(SqeStack *s);
int topStack(SqeStack *s);




int main()
{
    SqeStack *s = NULL;
    
    if ((s = initSqStack()) == NULL)   //注意segment fault问题
    {
        printf("初始化栈失败！ \n");
        return -1;
    } 

    pushStack(s, 10);
    pushStack(s, 20);
    pushStack(s, 30);
    pushStack(s, 40);

    stackShow(s);
    printf("返回栈顶元素：%d \n", topStack(s));
    popStack(s);
    freeStack(s);

    return 0;
}

int topStack(SqeStack *s)
{
    return s->stack[s->top];
}

/**
 * @description: 初始化栈，创建的第一个元素不存放数值
 * @param {type} 
 * @return: 
 */
SqeStack *initSqStack()
{
    SqeStack*s = NULL;
    s = (SqeStack *)malloc(sizeof(SqeStack));
    if (s == NULL)
    {
        printf("内存分配失败！ \n");
    }
    s->top = -1;   // 空栈
    return s;
}

/**
 * @description: 判断栈是否为空
 * @param {type} 
 * @return: 
 */
bool sqStackEmpty(SqeStack *s)
{
    if (s->top == -1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @description: 
 * @param {type} 求栈的长度
 * @return: 
 */
int sqStackLength(SqeStack *s)   
{
    return s->top + 1;   // 求栈的长度
}


/**
 * @description: 入栈操作 
 * @param {type} 
 * @return: 
 */
int pushStack(SqeStack *s, dataType value)
{
    if (s->top == (STACK_SIZE - 1))   // 判断栈是否满
    {
        return false;
    }
    else
    {
        s->top = s->top + 1;         // 栈顶指针移动  
        s->stack[s->top] = value;    //  栈顶指针出存放值a

    }
    return true;
}

/**
 * @description: 出栈操作
 * @param
 * input: 栈地址，出栈元素地址
 * @return: false：失败；true：操作正常
 */
void popStack(SqeStack *s)
{
    dataType value;
    int i = 0;
    while (s->top != -1)
    {
        value = s->stack[s->top];  // 记录栈顶元素值
        i = s->top +1;
        s->top--;                  // 栈顶依次出栈
        printf("出栈第 %d个元素为：%d \n", i, value);
    }
}

/**
 * @description: 打印栈元素
 * @param {type} 
 * @return: 
 */
void stackShow(SqeStack *s)
{
    int i;
    for(i = 0; i <= s->top; i++)
    {
        printf("%d \n", s->stack[i]);
    }
    printf("\n");
    // free(s);       // 释放内存
    // s = NULL;      // 指针置为空
}

/**
 * @description: 清除栈元素值，只保留第一个不放数值的元素
 * @param {type} 
 * @return: 
 */
void clearStack(SqeStack *s)
{
    s->top = -1;

}
void destoryStack(SqeStack *s)
{
    
}

/**
 * @description:  释放栈空间
 * @param {type} 
 * @return: 
 */
void freeStack(SqeStack *s)
{
    while (s->top != -1)
    {
        popStack(s);
    }
  printf("\n");
  free(s);
  s = NULL;
}

