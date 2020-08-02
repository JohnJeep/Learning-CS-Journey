/*
 * @Author: JohnJeep
 * @Date: 2020-07-22 22:17:04
 * @LastEditTime: 2020-08-02 11:12:47
 * @LastEditors: Please set LastEditors
 * @Description: 二叉树
 * @FilePath: /BinaryTree.c
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>

// 二叉链表结点表示
typedef struct tag_binNode{
    int data;
    struct tag_binNode *left, *right;
}BinTreeNode;

// 双亲链表结点表示
#define MAX_TREE_NODE    32
typedef struct tag_node
{
    int data;
    int parentPosition;  // 指向双亲的指针
    char lfTag;          // 左右孩子标志域
}BPNode;
typedef struct tag_tree
{
    BPNode nodes[MAX_TREE_NODE];    // 将结点存储在数组中
    int m_node;                     // 结点数
    int root;                       // 根节点位置，存储父结点所在的数组下标的位置
}BPTree;


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
 */
// 递归法实现
void preOrderTraverseRecursive(BinTreeNode *Tree)
{
    if (Tree != NULL)
    {
        printf("%d ", Tree->data);
        preOrderTraverseRecursive(Tree->left);
        preOrderTraverseRecursive(Tree->right);
    }
}

// 利用栈的思想实现
#define MAX_NODE    32
void preOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *stack[MAX_NODE];
    BinTreeNode *p = Tree;
    BinTreeNode *q = NULL;
    int top = 0;

    if (Tree == NULL)
    {   
        printf("binary tree is empty.\n");
        return;
    }
    else
    {
        do
        {
            /// visit(p->data);
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
// 递归法实现
void inOrderTraverseRecursive(BinTreeNode *Tree)
{
    if (Tree == NULL)
    {
        printf("binary tree is empty.\n");
        return;
    }
    
    inOrderTraverseRecursive(Tree->left);
    printf("in traverse: %d\n", Tree->data);
    inOrderTraverseRecursive(Tree->right);
}

void inOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *stack[MAX_NODE];
    BinTreeNode *p = Tree;
    int top = 0;
    int bl = 1;

    if (Tree == NULL)
    {   
        printf("binary tree is empty.\n");
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
                /// visit(p->data);
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
// 递归法实现
void postOrderTraverseRecursive(BinTreeNode *Tree)
{
    if (Tree == NULL)
    {
        printf("binary tree is empty.\n");
        return;
    }
    
    postOrderTraverseRecursive(Tree->left);
    postOrderTraverseRecursive(Tree->right);
    printf("%d\n", Tree->data);
}


void postOrderTraverse(BinTreeNode *Tree)
{
    BinTreeNode *s1[MAX_NODE];
    int s2[MAX_NODE];
    BinTreeNode *p = Tree;
    int top = 0;
    int bl = 1;

    if (Tree == NULL)
    {   
        printf("binary tree is empty.\n");
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
            /// visit(p->data);
            p = NULL;
        }
    } while (bl != 0);
    
}


/**
 * @description: 计算树的叶子结点
 * @param {type} 
 * @return: 
 * 思路：1、判断根结点是否为叶子结点
 *       2、遍历左子树的结点是否为叶子结点
 *       3、遍历右子树的结点是否为叶子结点
 * 
 */
void countLeafNodes(BinTreeNode* tree, int* sum)
{
    if (tree != NULL && sum != NULL)
    {
        if (tree->left == NULL && tree->right == NULL)
        {
            (*sum)++;
        }
        if (tree->left)
        {
            countLeafNodes(tree->left, sum);
        }
        if (tree->right)
        {
            countLeafNodes(tree->right, sum);
        }        
    }
}

/**
 * @description: 求树的深度，即结点子树的最大值加 1
 * @param {type} 
 * @return: 
 *  思路：1、求左子树和右子树的高度
 *        2、比较左右子树高度的大小，取两者的最大值并加 1
 * 
 */
int depth(BinTreeNode* tree)
{
    int val = 0;
    int leftDepth = 0;
    int rightDepth = 0;

    if (tree == NULL)
    {
        return val;
    }
    leftDepth = depth(tree->left);
    rightDepth = depth(tree->right);
    val = (leftDepth > rightDepth ? leftDepth:rightDepth) + 1;

    return val;
}

/**
 * @description: 拷贝一颗二叉树：一个结点一个结点的拷贝
 * @param {type} 
 * @return: 
 * 思路: 1、拷贝根结点
 *       2、分别拷贝左子树和右子树的结点
 *       3、使用一个新的结点连接拷贝的左子树和右子树
 *       4、将原来树的数据域拷贝到新树的数据域中
 */
BinTreeNode *copyTree(BinTreeNode* tree)
{
    BinTreeNode* leftTree = NULL;
    BinTreeNode* rightTree = NULL;
    BinTreeNode* newNode = NULL;

    if (tree == NULL)
    {
        printf("binary is NULL.\n");
        return NULL;
    }

    if (tree->left != NULL)
    {
        leftTree = copyTree(tree->left);
    }
    else
    {
        leftTree = NULL;
    }
    
    if (tree->right != NULL)
    {
        rightTree = copyTree(tree->right);
    }
    else
    {
        rightTree =NULL;
    }
    
    newNode = (BinTreeNode*)malloc(sizeof(BinTreeNode));
    if (newNode == NULL)
    {
        printf("allocate node failed.\n");
        return NULL;
    }
    newNode->data = tree->data;
    newNode->left = leftTree;
    newNode->right =rightTree;

    return newNode;    
}


/**
 * @description: 采用递归法用#号法创建一颗树    ----- 没有调通
 * @param {type} 
 * @return: 
 */
BinTreeNode* crateTree()
{
    BinTreeNode* node = NULL;
    char ch;

    printf("请输入字符：");
    scanf("%c", &ch);
    if (ch == '#')   // 根结点为 #
    {
        printf("create tree failed.\n");
        return NULL;
    }
    else
    {
        node = (BinTreeNode*)malloc(sizeof(BinTreeNode));
        if (node == NULL)
        {
            printf("allocate node failed.\n");
            return NULL;
        }
        node->data = ch;
        node->left = crateTree();
        node->right = crateTree();        
        return node;
    }
}

/**
 * @description: 释放一颗树分配的内存
 * @param {type} 
 * @return: 
 */
void freeTree(BinTreeNode* tree)
{
    if (tree == NULL)
    {
        printf("tree is NULL.\n");
    }
    if (tree->left != NULL)
    {
        free(tree->left);
        tree->left = NULL;
    }
    if (tree->right != NULL)
    {
        free(tree->right);
        tree->right = NULL;
    }
    if (tree != NULL)   // 最后释放根结点
    {
        free(tree);
        tree = NULL;
    }
}


// 测试用例
void test01(BinTreeNode root)
{
    printf("pre traverse: ");
    preOrderTraverseRecursive(&root);
    printf("\n");

    inOrderTraverseRecursive(&root);
    printf("\n");
    
    postOrderTraverseRecursive(&root);
    printf("\n");
}

void test02(BinTreeNode T)
{
    printf("\ntest case02!\n");
    int leafNode = 0;
    countLeafNodes(&T, &leafNode);
    printf("leafNode = %d\n", leafNode);
}

void test03(BinTreeNode T)
{
    printf("\ntest case03!\n");
    int dep = depth(&T);
    printf("dep = %d\n", dep);
}

void test04(BinTreeNode* T)
{
    printf("\ntest case04, copy a tree!\n");
    BinTreeNode* cpt = copyTree(T);
    printf("pre traverse: ");
    preOrderTraverseRecursive(cpt);
    printf("\n");
}

void test05()
{
    printf("\ntest case05, crate a tree!\n");
    BinTreeNode* ct = crateTree();
    printf("pre traverse: ");
    preOrderTraverseRecursive(ct);
    printf("\n");
}

int main()
{
    BinTreeNode t1, t2, t3, t4, t5;
    memset(&t1, 0, sizeof(BinTreeNode));
    memset(&t2, 0, sizeof(BinTreeNode));
    memset(&t3, 0, sizeof(BinTreeNode));
    memset(&t4, 0, sizeof(BinTreeNode));
    memset(&t5, 0, sizeof(BinTreeNode));

    t1.data = 1; 
    t2.data = 2; 
    t3.data = 3; 
    t4.data = 4; 
    t5.data = 5;
    t1.left = &t2;
    t1.right = &t3;
    t2.left = &t4;
    t3.right = &t5; 

    test01(t1);
    test02(t1);
    test03(t1);
    test04(&t1);
    test05();
    
   return 0;
}