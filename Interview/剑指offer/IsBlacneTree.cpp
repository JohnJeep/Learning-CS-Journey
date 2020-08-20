/*
 * @Author: JohnJeep
 * @Date: 2020-08-20 14:30:03
 * @LastEditTime: 2020-08-20 14:40:43
 * @LastEditors: Please set LastEditors
 * @Description: 题目：平衡二叉树
 *               描述：输入一棵二叉树，判断该二叉树是否是平衡二叉树。
 *               思路：
 *                   1、先求数的树的深度，根据平衡二叉树左右子树的深度绝对值之差小于等于 1 来判断
 *                   2、将二叉树中非二叉树子树的深度设置为 -1
 */
#include <iostream>
#include <cstdio>

using namespace std;

struct TreeNode
{
    int val;
    struct TreeNode* left;
    struct TreeNode* right;
    TreeNode(int x) :
            val(x), left(nullptr), right(nullptr){
             }
};

int treeDepth(TreeNode* pRoot)
{
    if (pRoot == nullptr)
    {
        return 0;
    }
    int left = treeDepth(pRoot->left);   
    if (left == -1)                       // 子树非不满足平衡二叉树的情况
    {
        return -1;
    }
    int right = treeDepth(pRoot->right);
    if (right == -1)
    {
        return -1;
    }
    if (abs(left - right) > 1)
    {
        return -1;
    }
    return (left > right ? left : right) + 1;
}

bool IsBalanced_Solution(TreeNode* pRoot) 
{
    if (treeDepth(pRoot) != -1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int main(int argc, char *argv[])
{
    
    return 0;
}