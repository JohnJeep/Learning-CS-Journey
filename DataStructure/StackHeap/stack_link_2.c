#include <stdio.h>
#include <malloc.h>

typedef int dataType;
typedef struct node
{
	dataType data;
	struct node *next;   //��ָ��
}LinkStack;


/*
���ܣ���ջ����
���룺ջ��ָ�룬��ջԪ��
�����ջ��ָ��
*/
LinkStack *PushStack(LinkStack *top, dataType x)
{
	LinkStack *p;
	p = malloc(sizeof(LinkStack)); //�ڵ�p����ռ�
	p->data = x;
	p->next = top;
	top = p;			//�޸�ջ��ָ��
	return top;         //����ջ��ָ��
}


/*
���ܣ���ջ����
���룺ջ��ָ�룬��ջԪ��
�����ջ��ָ��
*/
LinkStack *Pop(LinkStack *top, dataType *datap)
{
	LinkStack *p;
	if (top != NULL)
	{
		*datap = top->data;  //��¼ջ���ڵ��ַ��Ԫ��ֵ
		p = top;
		top = top->next;     //�޸�ջ��ָ��top
		free(p);             //�ͷŽڵ�p
	}
	return top;              //����ջ��ָ��
}

int main()
{
	LinkStack *topPtr;  //ջ��ָ��

	topPtr = PushStack(NULL, 'A');
	topPtr = PushStack(&topPtr, 'B');

	return 0;

}







