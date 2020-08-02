/*
 * @Author: JohnJeep
 * @Date: 2020-08-01 22:08:51
 * @LastEditTime: 2020-08-02 00:12:48
 * @LastEditors: Please set LastEditors
 * @Description: 典型的二叉树中序遍历非递归实现
 * @FilePath: /InOrderTraverse.cpp
 */ 
#include <iostream>
#include <stdlib.h>
#include <stack>
#include <memory.h>

using namespace std;

typedef struct tag_binNode{
    int data;
    struct tag_binNode *left, *right;
}BinTreeNode;


/**
 * @description: 一直遍历左边的结点，直到找到中序遍历的起点
 * @param {type} 
 * @return: 
 */
BinTreeNode* traverseLeft(BinTreeNode* tree, stack<BinTreeNode*>& st)
{
    if (tree != NULL)
    {
        while (tree->left != NULL)    
        {
            st.push(tree);               // 左子树可能是子树的集合
            tree = tree->left;
        }
    }
    
    return tree;
}

/**
 * @description: 非递归中序遍历函数主体
 * @param {type} 
 * @return: 
 */
void inOrderTraverse(BinTreeNode* tree)
{
    BinTreeNode* p = NULL;
    stack<BinTreeNode*> s;

    if (tree == NULL)
    {
        printf("Binary is NULL.\n");
    }
    
    p = traverseLeft(tree, s);
    while (p)
    {
        printf("%d\n", p->data);   // 1、得到中序遍历的根结点
        if (p->right != NULL)      // 2、若有右子树集，重复一直向左遍历的步骤，得到右子树集合中序遍历的起点
        {
            p = traverseLeft(p->right, s);
        }
        else if (!s.empty())       // 3、若没有右子树集，根据栈顶指针进行相应的出栈操作
        {
            p = s.top();
            s.pop();
        }
        else
        {
            p = NULL;              // 4、若没有右子树集且栈为空时，整个中序遍历结束
        }
    }
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

    inOrderTraverse(&t1);
    
   return 0;
}