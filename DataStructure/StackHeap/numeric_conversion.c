/*
利用堆栈实现任意十进制的数转换为任意R进制的数，
求的第一位余数放在栈的最低位，最后一位余数放在栈的最高位
*/
#include <stdio.h>

#define STACK_SIZE 6
typedef int dataType;
typedef struct Stack
{
	dataType stack[STACK_SIZE];
	int top;
}SeqStack;


void InitStack(SeqStack *s)
{
	//初始化栈顶为空
	s->top = -1;
}

//判断栈是否为空
int StackEmpty(SeqStack *s)
{
	return s->top == -1;
}

int Push(SeqStack *s, dataType x)
{
	if (s->top == STACK_SIZE)
		return 0;
	else
	{
		s->top++;
		s->stack[s->top] = x;
	}
	return 1;
}

int Pop(SeqStack *s, dataType *x)
{
	if (s->top == -1)
		return 0;
	else
	{
		*x = s->stack[s->top];
		s->top--;
	}
	return 1;
}

//任意的一个十进制数转换为R进制的数
void Convert(SeqStack *s, int n)
{
	int m;
	while (n) //非零值
	{
		Push(s, n % 2);
		n = n / 2;         //非零商继续运算
	}
	while (!StackEmpty(s))
	{
		Pop(s, &m);
		printf("%d \n", m);
	}
}


int main()
{
	SeqStack s;
	InitStack(&s);
	Convert(&s, 4);
	return 0;
}