//�ڶ��ֽṹ��ջ����


#include <stdio.h>

#define STACK_SIZE 6   //ջ��������
typedef char dataType;

typedef struct Stack
{
	dataType stack[STACK_SIZE];//ջ�ռ�
	int top;   //ջ��ָ��
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

//ջ��ʼ��
void InitStack(SeqStack *s)
{
	s->top = -1;               //ջΪ��
}


/*
���ܣ���ջ����
���룺ջ��ַ����ջԪ��ֵ
���������----ջ��ַ�����----NULL
*/
SeqStack *Push(SeqStack *s, dataType x)
{
	if (s->top > STACK_SIZE - 1)
	{
		printf("ջ���");
		return NULL;
	}
	else
	{
		s->top++;
		s->stack[s->top] = x;
	}

	//ע�⣺�ڽ�ջ������s�ĵ�ַû�б仯�����÷���ֵû��Ҫ�������Ʋ����
	return s;
}