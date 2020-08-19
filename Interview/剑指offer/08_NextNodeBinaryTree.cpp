/*
 * @Author: JohnJeep
 * @Date: 2020-07-28 22:09:46
 * @LastEditTime: 2020-08-09 18:55:28
 * @LastEditors: Please set LastEditors
 * @Description: 题目：给定一个二叉树和其中的一个结点，请找出中序遍历顺序的下一个结点并且返回。
 *                     注意，树中的结点不仅包含左右子结点，同时包含指向父结点的指针。
 * 
 *               思路：判断当前结点是否有右子树?
 *                     1、有右子树---->判断右子树下是否有左子树？
 *                           1）没有---->则当前结点的下一个结点就是它右子树中的最左子结点
 *                           2）有  ---->遍历左子树
 *                     2、无右子树
 *                           1）当前结点为根结点时，则直接返回 nullptr
 *                           2) 当前结点不为根结点，又要判断当前结点是它父结点的左孩子还是右孩子
 *                               a、左孩子--->直接返回它的父结点
 *                               b、右孩子--->向上遍历它的祖先结点，直到找到一个祖先结点满足为左孩子的条件，否则返回null，当前结点为尾结点
>                                   
 * 
 * 
 * @FilePath: /08_NextNodeBinaryTree.cpp
 */ 
#include <iostream>
#include <stdlib.h>

using namespace std;

struct TreeLinkNode {
    int val;
    struct TreeLinkNode *left;
    struct TreeLinkNode *right;
    struct TreeLinkNode *next;
    TreeLinkNode(int x) :val(x), left(NULL), right(NULL), next(NULL) {
        
    }
};

class Solution {
public:
    TreeLinkNode* GetNext(TreeLinkNode* pNode)
    {
        if (pNode == nullptr)
        {
             return nullptr;
        }
        TreeLinkNode* pNext = nullptr;         // 要返回的结点

        if (pNode->right != nullptr)           // 有右子树
        {
            TreeLinkNode* leftNode = pNode->right;
            while (leftNode->left != nullptr)  // 右子树下有左子树
            {
                leftNode = leftNode->left;     // 遍历左子树
            }
            pNext = leftNode;                  // 右子树下没有左子树
        }
        else if (pNode->next == nullptr)       // 无右子树且当前结点为根结点
        {
            return nullptr;
        }
        else if(pNode->next != nullptr)        // 无右子树且当前结点不为根结点
        {
            TreeLinkNode* pParent = pNode->next; // 当前结点的父结点
            while ((pParent != nullptr) && (pNode == pParent->right))  // 有右子树，则向上遍历父结点中是否有结点满足一个结点为左子树
            {
                pNode = pParent;
                pParent = pParent->next;
            }
            pNext = pParent;                   // 是左子树时 
        }
        return pNext;
    }
};

