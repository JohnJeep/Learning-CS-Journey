/*
 * @Author: JohnJeep
 * @Date: 2020-08-20 14:45:32
 * @LastEditTime: 2020-08-21 15:26:03
 * @LastEditors: Please set LastEditors
 * @Description: 题目：二叉搜索树的第 k 个结点
 *               描述：给定一棵二叉搜索树，请找出其中的第k小的结点。例如，(5，3，7，2，4，6，8)中，按结点数值大小顺序第三小结点的值为4。
 *               思路：利用二叉树的中序遍历实现
 * 
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

// 递归实现，k是一个全局的变量，存储的是当前需要搜索的结点
TreeNode* InorderTraverse(TreeNode* pRoot, unsigned int& k)   // 注意  unsigned int& k 中的&符号
{
    TreeNode* target = nullptr;
    
    if (pRoot->left != nullptr)
    {
        target = InorderTraverse(pRoot->left, k);
    }
    if (target == nullptr)
    {
        if (k == 1)  // 二叉树为空树时
        {
            target = pRoot;
        }
        k--;         // 二叉树不为空树时
    }
    if (target == nullptr && pRoot->right != nullptr)
    {
        target = InorderTraverse(pRoot->right, k);
    }

    return target;
}

TreeNode* KthNode(TreeNode* pRoot, unsigned int k)
{
    if (pRoot == nullptr || k <= 0)
    {
        return nullptr;
    } 
    return InorderTraverse(pRoot, k);
}
int main(int argc, char *argv[])
{
    
    return 0;
}