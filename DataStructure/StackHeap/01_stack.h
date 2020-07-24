/*
 * @Author: JohnJeep
 * @Date: 2020-07-22 21:20:00
 * @LastEditTime: 2020-07-24 11:13:02
 * @LastEditors: Please set LastEditors
 * @Description: stack的底层接口
 * @FilePath: /01_stack.h
 */ 
#ifndef __01_STACK_H
#define __01_STACK_H

typedef int elemType;
typedef struct tag_sqStack
{
    elemType *bottom;
    elemType *top;
    int stackSize;
}sqStack;



typedef void Stack;
void stackInit();                            // 栈的初始化
void stackDestory(Stack* stack);             // 销毁栈
void* stackTop(Stack* stack);                // 取栈顶元素
int stackPush(Stack* stack, void* iterm);    // 进栈
void* stackPop(Stack* stack);                // 出栈
void stackClear(Stack* stack);               // 清空栈
int stackSize(Stack* stack);                 // 获取栈的大小

#endif // !__01_STACK_H
