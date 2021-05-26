#include <stdio.h>
#include <malloc.h>

typedef int dataType;
typedef struct node
{
	dataType data;
	struct node *next;   //链指针
}LinkStack;


/*
功能：进栈操作
输入：栈顶指针，进栈元素
输出：栈顶指针
*/
LinkStack *PushStack(LinkStack *top, dataType x)
{
	LinkStack *p;
	p = malloc(sizeof(LinkStack)); //节点p申请空间
	p->data = x;
	p->next = top;
	top = p;			//修改栈顶指针
	return top;         //返回栈顶指针
}


/*
功能：出栈操作
输入：栈顶指针，出栈元素
输出：栈顶指针
*/
LinkStack *Pop(LinkStack *top, dataType *datap)
{
	LinkStack *p;
	if (top != NULL)
	{
		*datap = top->data;  //记录栈顶节点地址和元素值
		p = top;
		top = top->next;     //修改栈顶指针top
		free(p);             //释放节点p
	}
	return top;              //返回栈顶指针
}

int main()
{
	LinkStack *topPtr;  //栈顶指针

	topPtr = PushStack(NULL, 'A');
	topPtr = PushStack(&topPtr, 'B');

	return 0;

}







