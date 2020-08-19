/*
 * @Author: your name
 * @Date: 2020-07-28 22:08:31
 * @LastEditTime: 2020-08-09 09:52:10
 * @LastEditors: Please set LastEditors
 * @Description: 重建二叉树：输入某二叉树的前序遍历和中序遍历的结果，请重建出该二叉树。假设输入的前序遍历和中序遍历的结果中都不含重复的数字。
 * 
 * @FilePath: /07_ConstructBinaryTree.cpp
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
    
    int rootValue = preStart[0];
    TreeNode* root = new TreeNode(preStart[0]);
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
    Solution<vector<int>> binTree;

    // binTree.reConstructBinaryTree();

    return 0;
}