/*栈与堆空间地址的传递*/
#include<stdio.h> 
#include<malloc.h>


typedef struct node
{
	int data;
	struct node *next;
}LinkListNode;

LinkListNode *CreateNode(void)//指针函数 
{
	LinkListNode *p, *heap, stack;

	//heap的指向空间通过malloc函数申请，在堆中分配 
	heap = (LinkListNode*)malloc(sizeof(LinkListNode));
	heap->data = 8;//heap的数据域 
	heap->next = NULL;

	//stack变量空间在栈中分配 
	stack = *heap;//heap节点内容复制费stack节点 
	p = &stack;   //返回栈空间地址
	//	p = heap;	  //返回堆空间地址 
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