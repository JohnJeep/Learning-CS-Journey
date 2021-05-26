/*
���ö�ջʵ������ʮ���Ƶ���ת��Ϊ����R���Ƶ�����
��ĵ�һλ��������ջ�����λ�����һλ��������ջ�����λ
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
	//��ʼ��ջ��Ϊ��
	s->top = -1;
}

//�ж�ջ�Ƿ�Ϊ��
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

//�����һ��ʮ������ת��ΪR���Ƶ���
void Convert(SeqStack *s, int n)
{
	int m;
	while (n) //����ֵ
	{
		Push(s, n % 2);
		n = n / 2;         //�����̼�������
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