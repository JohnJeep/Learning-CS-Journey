//第二种结构入栈操作


#include <stdio.h>

#define STACK_SIZE 6   //栈容量设置
typedef char dataType;

typedef struct Stack
{
	dataType stack[STACK_SIZE];//栈空间
	int top;   //栈顶指针
}SeqStack;


void InitStack(SeqStack *s);
SeqStack *Push(SeqStack *s, dataType x);

int main()
{
	SeqStack sa, *sPtr;


	InitStack(&sa);
	sPtr = Push(&sa, 'a');
	sPtr = Push(&sa, 'b');

	return 0;
}

//栈初始化
void InitStack(SeqStack *s)
{
	s->top = -1;               //栈为空
}


/*
功能：进栈操作
输入：栈地址，进栈元素值
输出：正常----栈地址，溢出----NULL
*/
SeqStack *Push(SeqStack *s, dataType x)
{
	if (s->top > STACK_SIZE - 1)
	{
		printf("栈溢出");
		return NULL;
	}
	else
	{
		s->top++;
		s->stack[s->top] = x;
	}

	//注意：在进栈过程中s的地址没有变化，设置返回值没必要，框架设计不简洁
	return s;
}