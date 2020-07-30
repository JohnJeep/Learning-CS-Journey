/*
 * @Author: JohnJeep
 * @Date: 2020-07-22 22:17:04
 * @LastEditTime: 2020-07-30 14:34:41
 * @LastEditors: Please set LastEditors
 * @Description: 二叉树
 * @FilePath: /binaryTree.c
 * 
 */
#include <stdio.h>
#include <string.h>
#include <malloc.h>

typedef struct tag_binNode{
    int data;
    struct tag_binNode *left, *right;
}BinTreeNode;

/*
 *                             前序遍历思路
 * 法一：利用递归遍历
 *       若二叉树为空，则遍历结束；否则
 *       ⑴ 访问根结点；
 *       ⑵ 先序遍历左子树(递归调用本算法)；
 *       ⑶ 先序遍历右子树(递归调用本算法)。
 * 
 * 法二：非递归，利用栈的思想
 *       若二叉树为空，则返回；否则，令p=T；
 *       ⑴ 访问p所指向的结点；
 *       ⑵ q=p->Rchild ，若q不为空，则q进栈；
 *       ⑶ p=p->left ，若p不为空，转(1)，否则转(4)；
 *       ⑷  退栈到p ，转(1)，直到栈空为止。
 * 
 * 
 */
#define MAX_NODE    32
void preOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *stack[MAX_NODE];
    BinTreeNode *p = Tree;
    BinTreeNode *q = NULL;
    int top = 0;

    if (Tree == NULL)
    {   
        printf("binay tree is empty.\n");
        return;
    }
    else
    {
        do
        {
            visit(p->data);
            q = p->right;
            if (q != NULL)
            {
                stack[++top] = q;
            }
            p = p->left;
            if (p == NULL)
            {
                p = stack[top];
                top--;
            }
        } while (p != NULL);
    }
}


/*
 *                             中序遍历思路
 * 法一：利用递归遍历
 *       若二叉树为空，则遍历结束
 *       ⑴ 访问左结点(递归调用本算法)；
 *       ⑵ 中序遍历根子树；
 *       ⑶ 中序遍历右子树(递归调用本算法)。
 * 
 * 法二：非递归，利用栈的思想
 *       若二叉树为空，则返回；否则，令p=Tree
 *       ⑴ 若p不为空，p进栈， p=p->left ；
 *       ⑵ p为空，退栈到p，访问p所指向的结点；
 *       ⑶ p=p->Rchild ，转(1)；直到栈空为止。
 * 
 */
void inOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *stack[MAX_NODE];
    BinTreeNode *p = Tree;
    int top = 0;
    int bl = 1;

    if (Tree == NULL)
    {   
        printf("binay tree is empty.\n");
        return;
    }
    else
    {
        do
        {
            while (p != NULL)
            {
                stack[++top] = p;
                p = p->left;
            }
            if (top == 0)
            {
                bl = 0;
            }
            else
            {
                p = stack[top];
                top--;
                visit(p->data);
                p = p->right;
            }
        } while (bl != 0);
    }
}


/*
 *                            后序遍历思路
 * 法一：利用递归遍历
 *       若二叉树为空，则遍历结束；否则
 *       ⑴ 访问左结点(递归调用本算法)；
 *       ⑵ 后序遍历右子树(递归调用本算法)；
 *       ⑶ 后序遍历根子树。
 * 
 * 法二：非递归，利用栈的思想
 *       若二叉树为空，则返回；否则，令p=Tree；
 *       ⑴ 第一次经过根结点p，不访问：p进栈S1 ， tag 赋值0，进栈S2，p=p->left 。
 *       ⑵ 若p不为空，转(1)，否则，取状态标志值tag ：
 *       ⑶ 若tag=0：对栈S1，不访问，不出栈；修改S2栈顶元素值(tag赋值1) ，取S1栈顶元素的右子树，即p=S1[top]->Rchild ，转(1)；
 *       ⑷ 若tag=1：S1退栈，访问该结点；直到栈空为止。
 * 
 */
void postOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *s1[MAX_NODE];
    int s2[MAX_NODE];
    BinTreeNode *p = Tree;
    int top = 0;
    int bl = 1;

    if (Tree == NULL)
    {   
        printf("binay tree is empty.\n");
        return;
    }
    do
    {
        while (p !=NULL)
        {
            s1[++top] = p;
            s2[top] = 0;
            p = p->left;
        }
        if (top == 0)
        {
            bl = 0;
        }
        else if (s2[top] == 0)
        {
            p = s1[top]->right;
            s2[top] = 1;
        }
        else
        {
            p = s1[top];
            top--;
            visit(p->data);
            p = NULL;
        }
    } while (bl != 0);
    
}



int main()
{

   return 0;
}