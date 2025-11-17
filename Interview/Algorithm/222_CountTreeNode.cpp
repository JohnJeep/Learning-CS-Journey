/*
 * @Author: JohnJeep
 * @Date: 2020-08-21 11:31:30
 * @LastEditTime: 2020-08-21 11:58:44
 * @LastEditors: Please set LastEditors
 * @Description: 题目：统计完全二叉树多少个节点
 *               描述：Given a complete binary tree, count the number of nodes.
 *               Example: Input
 *                             1
 *                            / \
 *                           2   3
 *                          / \  /
 *                         4  5 6
 *                            
 *                        Output: 6
 *      
 *               思路：
 *                  1、二叉树的高度为h，当完全二叉树为满二叉树时，结点总数为 2的h次方减1
 *                  2、当为满二叉树时，结点总数为左右子树的结点总数之和再加根结点。
 *                  3、左右子树的结点个数通过递归来计算
 */
#include <iostream>
#include <cstdio>
#include <cmath>

using namespace std;

struct TreeNode {
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode(int x) : val(x), left(nullptr), right(nullptr) {}          // 结构体参数初始化
};


int countTreeNodes(TreeNode* root)
{
    int leftHigh = 0;
    int rightHigh = 0;
    TreeNode* leftTree = root;
    TreeNode* rightTree = root;
    while (leftTree)
    {
        leftHigh++;
        leftTree = leftTree->left;
    }
    while (right)
    {
        rightHigh++;
        rightTree= rightTree->right;
    }
    if (leftHigh == rightHigh)
    {
        return pow(2, leftHigh) - 1;
    }
    
    return (countTreeNodes(root->left) + countTreeNodes(root->right) + 1);
}

int main(int argc, char *argv[])
{
    
    return 0;
}

