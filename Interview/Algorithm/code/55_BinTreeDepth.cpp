/*
 * @Author: JohnJeep
 * @Date: 2020-08-20 11:58:58
 * @LastEditTime: 2020-08-20 14:44:14
 * @LastEditors: Please set LastEditors
 * @Description: 题目：二叉树的深度
 *               描述：
 *               思路：
 *                  1、二叉树为空树时，深度为 0
 *                  2、利用的递归的方法分别遍历左子树和右子树，取左右子树深度的最大值
 *                  3、再左右子树深度的最大值的结果上 加上根节点，得到二叉树的深度
 * 
 */
#include <iostream>
#include <cstdio>

using namespace std;

struct TreeNode {
	int val;
	struct TreeNode *left;
	struct TreeNode *right;
	TreeNode(int x) :
			val(x), left(NULL), right(NULL) {
	}
};

int TreeDepth(TreeNode* pRoot)
{
    if (pRoot == nullptr)
    {
        return 0;
    }
    int left = TreeDepth (pRoot->left);
    int right = TreeDepth (pRoot->right);
    if (left > right)
    {
        return left + 1;
    }
    else
    {
        return right + 1;
    }
    // 另外一种写法：return (left > right ? left : right) + 1;
}

int main(int argc, char *argv[])
{
    
    return 0;
}
