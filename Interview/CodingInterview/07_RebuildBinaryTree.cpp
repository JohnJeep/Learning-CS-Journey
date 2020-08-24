/*
 * @Author: JOhnJeep
 * @Date: 2020-07-28 22:08:31
 * @LastEditTime: 2020-08-21 10:57:19
 * @LastEditors: Please set LastEditors
 * @Description: 题目：重建二叉树
 *               描述：输入某二叉树的前序遍历和中序遍历的结果，请重建出该二叉树。
 *                     假设输入的前序遍历和中序遍历的结果中都不含重复的数字。
 * 
 *               思路：
 *                  1、找到根节点。前序遍历的第一个结点是根节点的值。
 *                  2、在中序遍历中找到根节点的位置，根结点左边为左子树，右边为右子树。
 *                  3、递归调用：将左子树和右子树分别看成一颗树，将其前序遍历序列、中序遍历序列分别传入到该函数中，
 *                     便可得到左子树的根结点、右子树的根结点，需要用第一步得到的根结点连接它们。
 *                  4、递归条用中止的条件：传入的结点为nullptr时
 * 
 */ 
#include <iostream>
#include <stdlib.h>
#include <vector>

using namespace std;

struct TreeNode {
    int val;
    TreeNode *left;
    TreeNode *right;
    TreeNode(int x) : val(x), left(NULL), right(NULL) {}          // 结构体参数初始化
};

TreeNode* constructBody(int *preStart, int *preEnd, int *vinStart, int *vinEnd)
{
    if (preStart == nullptr || preStart == nullptr || preStart == nullptr || preStart == nullptr)
    {
        return nullptr;
    }
    
    // 前序遍历的第一个结点是根节点
    int rootValue = preStart[0];      
    TreeNode* root = new TreeNode(preStart[0]);   // 创建根节点
    root->val = rootValue;
    root->right = nullptr;
    root->left = nullptr;
    if (preStart == preEnd)
    {
        if (preStart == vinEnd && *preStart == *vinStart)
        {
            return root;
        }
    }
    
    // 在中序遍历中找到根节点的值
    int *vinRoot = vinStart;
    while (vinRoot <= vinEnd && *vinRoot != rootValue)
    {
        ++vinRoot;
    }
    
    int leftLen = vinRoot - vinStart;
    int *preLeftEnd = preStart + leftLen;
    if (leftLen > 0)
    {
        root->left = constructBody(preStart + 1, preLeftEnd, vinStart, vinRoot - 1); // 构建左子树
    }
    if (leftLen < preEnd - preStart)
    {
        root->right = constructBody(preLeftEnd + 1, preEnd, vinRoot + 1, vinEnd);  // 构建右子树
    }
    
    return root;
}

template <typename T>
class Solution {
public:
    TreeNode* reConstructBinaryTree(vector<int> pre,vector<int> vin) {
        if (pre.size() == 0 || vin.size() == 0)
        {
            return nullptr;
        }
        int *preStart = &pre[0];
        int *preEnd = &pre[pre.size() - 1];
        int *vinStart = &vin[0];
        int *vinEnd = &vin[vin.size() - 1];
        
        return constructBody(preStart, preEnd, vinStart, vinEnd);
    }
};


int main(int argc, char *argv[])
{
    return 0;
}