/*ջ��ѿռ��ַ�Ĵ���*/
#include<stdio.h> 
#include<malloc.h>


typedef struct node
{
	int data;
	struct node *next;
}LinkListNode;

LinkListNode *CreateNode(void)//ָ�뺯�� 
{
	LinkListNode *p, *heap, stack;

	//heap��ָ��ռ�ͨ��malloc�������룬�ڶ��з��� 
	heap = (LinkListNode*)malloc(sizeof(LinkListNode));
	heap->data = 8;//heap�������� 
	heap->next = NULL;

	//stack�����ռ���ջ�з��� 
	stack = *heap;//heap�ڵ����ݸ��Ʒ�stack�ڵ� 
	p = &stack;   //����ջ�ռ��ַ
	//	p = heap;	  //���ضѿռ��ַ 
	return p;
}

int main()
{
	LinkListNode *head, *x;
	int y;
	head = CreateNode();
	x = head;
	printf("%x:%d%d \n", x, x->data, x->next);
	y = x->data;
	printf("%x:%d%d \n", x, y, x->next);
	return 0;
}