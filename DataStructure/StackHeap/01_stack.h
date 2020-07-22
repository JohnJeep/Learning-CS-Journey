/*
 * @Author: JohnJeep
 * @Date: 2020-07-22 21:20:00
 * @LastEditTime: 2020-07-22 21:39:30
 * @LastEditors: Please set LastEditors
 * @Description: stack的底层接口
 * @FilePath: /01_stack.h
 */ 
#ifndef __01_STACK_H
#define __01_STACK_H

typedef void Stack;
void stackInit();                            // 栈的初始化
void stackDestory(Stack* stack);             // 销毁栈
void* stackTop(Stack* stack);                // 取栈顶元素
int stackPush(Stack* stack, void* iterm);    // 进栈
void* stackPop(stack* stack);                // 出栈
void stackClear(Stack* stack);               // 清空栈
int stackSize(Stack* stack);                 // 获取栈的大小

#endif // !__01_STACK_H
