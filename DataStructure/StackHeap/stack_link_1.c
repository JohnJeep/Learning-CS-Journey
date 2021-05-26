#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <stdbool.h>

typedef int dataType;

// 定义结点的结构体
typedef struct node
{
    dataType data;
    struct node *next;
}LinkNode, *PLinkNode;

// 定义栈顶与栈底的结构体
typedef struct stack
{
    PLinkNode top;
    PLinkNode buttom;
}Stack, *PStack;


void initLinkStack(PStack s);
bool pushLinkStack(Stack *s, int value);
bool popLinkStack(Stack *s, int value);
void traverseLinkStack(Stack *s);
bool emptyLinkStack(Stack *s);
void clearLinkStack(Stack *s);



int main()
{
    PStack s;
    int value;
    initLinkStack(s);   //产生空栈
    pushLinkStack(s, 1);
    pushLinkStack(s, 2);
    pushLinkStack(s, 3);
    pushLinkStack(s, 4);
    pushLinkStack(s, 5);
    pushLinkStack(s, 6);
    traverseLinkStack(s);
    
    if (popLinkStack(s, value))
    {
        printf("出栈成功，出栈元素为：%d \n", value);
    }
    else
    {
        printf("出栈失败");
    }
    
    clearLinkStack(s);
    traverseLinkStack(s);

    return 0;
}

/**
 * @description: 初始化链栈 
 * @param {type} 
 * @return: 
 */
void initLinkStack(PStack s)
{
    s->top = (LinkNode *)malloc(sizeof(LinkNode));
    if (s->top == NULL)
    {
        printf("内存分配失败！\n");
        exit(-1);
    }
    else
    {
        s->buttom = s->top;
        s->top->next = NULL;     //  栈底的指针域为空
    }
}

/**
 * @description: 遍历栈元素 
 * @param {type} 
 * @return: 
 */
void traverseLinkStack(Stack *s)
{
    PLinkNode p = s->top;     // 定义指针p指向栈顶
    while(p != s->buttom)
    {
        printf("%d", p->data);  // 遍历的值
        p = p->next;   // 移动p指针
    }
    printf("\n");
}

/**
 * @description: 判断栈是否为空
 * @param {type} 
 * @return: 
 */
bool emptyLinkStack(Stack *s)
{
    if (s->top == s->buttom)
    {
        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @description: 栈清空
 * @param {type} 
 * @return: 
 */
void clearLinkStack(Stack *s)
{
    if (emptyLinkStack(s))    // s中存放的本身就是s的地址
    {
        return;
    }
    else
    {
        PLinkNode p = s->top; // 定义指针p指向栈顶
        PLinkNode q = NULL;
        while (p != s->buttom)
        {
            q = p->next;   // q指针指向P的指针域
            free(p);
            p = q;       //移动指针p
        }
        s->top = s->buttom;    //栈为空
    }
}

/**
 * @description: 进栈操作
 * @param {type} 
 * @return: 
 */
bool pushLinkStack(Stack *s, int value)
{
    PLinkNode p;
    p = (PLinkNode)malloc(sizeof(PLinkNode));
    p->data = value;
    p->next = s->top;
    s->top = p;    // 栈顶指针指向指针，移动栈
    return true;
}


/**
 * @description:出栈操作 
 * @param {type} 
 * @return: 
 */
bool popLinkStack(Stack *s, int value)
{
    if (emptyLinkStack(s))    // s中存放的本身就是s的地址
    {
        return true;
    }
    else
    {
        LinkNode *p;
        p = s->top;
        value = p->data;
        s->top = p->next;    ///
        free(p);
        return true;
    }
}







// PLinkNode pushLinkStack(LinkNode *top)
// {
//     LinkNode *p;
//     dataType x;
//     p = (LinkNode *)malloc(sizeof(LinkNode));
//     p->data = x;
//     p->next = top;
//     top = p;                // 栈顶指针指向指针，移动栈
//     return p;
// }

// PLinkNode popLinkStack(LinkNode *top)
// {
//     LinkNode *p;
//     dataType value;
//     if (top != NULL)
//     {
//         value = p->data;
//         p = top;            // 指针p指向top指针，移动p指针
//         top = top->next;
//         free(p);
//     }
//     return p;
// }









