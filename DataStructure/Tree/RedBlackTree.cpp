/*
 * @Author: JohnJeep
 * @Date: 2020-09-04 08:50:31
 * @LastEditTime: 2020-09-07 22:37:12
 * @LastEditors: Please set LastEditors
 * @Description: 红黑树的理解
 * 
 */
#include <iostream>
#include <cstdio>
#include <cassert>

using namespace std;

enum rb_color
{
    RB_BLACK,
    RB_RED
};

struct Node
{
    Node* parent;
    Node* left;
    Node* right;
    enum rb_color color;
    int key;
};

Node* getParent(Node* p)
{
    return p == nullptr ? nullptr : p->parent;
}

Node* getGrandParent(Node* p)
{
    // Note that it will return nullptr if this is root or child of root
    return getParent(getParent(p));
}

// 得到兄弟姊妹结点
Node* getSibling(Node* p)
{
    Node* pNode = getParent(p);

    if (pNode == nullptr)
    {
        return nullptr;
    }
    if (p == pNode->left)
    {
        return pNode->right;
    }
    else
    {
        return pNode->left;
    }
}

Node* getUncle(Node* p)
{
    Node* pNode = getParent(p);

    return getSibling(pNode);
}

void rotateLeft(Node* p)
{
    Node* nnew = p->right;
    Node* pNode = getParent(p);
    assert(nnew != nullptr);    // 红黑树的叶子结点为空时，不会成为内部结点

    p->right = nnew->left;
    nnew->left = p;
    p->parent = nnew;

    if (p->right != nullptr)
    {
        p->right->parent = p;
    }
    
    // initially p node could be the root
    if (pNode != nullptr)
    {
        if (p == pNode->left)
        {
            pNode->left = nnew;
        }
        else if (p == pNode->right)
        {
            pNode->right = nnew;
        }
    }
    nnew->parent = pNode;    
}

void rotateRight(Node* p)
{
    Node* nnew = p->left;
    Node* pNode = getParent(p);
    assert(nnew != nullptr);    // 红黑树的叶子结点为空时，不会成为内部结点

    p->left = nnew->right;
    nnew->right = p;
    p->parent = nnew;

    if (p->left != nullptr)
    {
        p->left->parent = p;
    }
    
    // initially p node could be the root
    if (pNode != nullptr)
    {
        if (p == pNode->left)
        {
            pNode->left = nnew;
        }
        else if (p == pNode->right)
        {
            pNode->right = nnew;
        }
    }
    nnew->parent = pNode;    
}    

Node* insert(Node* root, Node* p)
{
    insertRecurse(root, p); // 往当前树中插入一个新的结点

    // 新节点插入后破坏红黑树的性质，需要进行修复来确保红黑树的性质
    insertRepairTree(p);
    
    // 找到新的root 就返回
    root = p;
    while (getParent(root) != nullptr)
    {
        root = getParent(root);
    }
    return root;
}

void insertRecurse(Node* root, Node* p)
{
    // 尾部递归遍历树的后代，直到出现叶子结点为止
    if (root != nullptr)
    {
        if (p->key < root->key)
        {
            if (root->left != nullptr)
            {
                insertRecurse(root->left, p);
                return;
            }
            else
            {
                root->left = p;
            }
        }
        else
        {
            if (root->right != nullptr)
            {
                insertRecurse(root->right, p);
                return;
            }
            else
            {
                root->right = p;
            }
        }
    }

    // 插入新的结点p
    p->parent = root;
    p->left = nullptr;
    p->right = nullptr;
    p->color = RB_RED;
}

void insertRepairTree(Node* p)
{
    if (getParent(p) == nullptr)
    {
        insertCase1(p); // p is the root node, first node of red–black tree
    }
    else if (getParent(p)->color == RB_BLACK)
    {
        insertCase2(p);  // p's parent is black
    }
    else if (getUncle(p) != nullptr && getUncle(p)->color == RB_BLACK)
    {
        insertCase3(p);  // parent is red and uncle is red      
    }
    else
    {
        insertCase4(p); // parent is red and uncle is black
    }
}

void insertCase1(Node* p)
{
    p->color = RB_BLACK;   // 违反性质5
}

void insertCase2(Node* p)
{
    return;    // 满足所有的性质
}

void insertCase3(Node* p)
{
    // 违反性质2 或 违反性质4
    getParent(p)->color = RB_BLACK;
    getUncle(p)->color = RB_BLACK;
    getGrandParent(p)->color = RB_RED;
    insertRepairTree(getGrandParent(p));
}

void insertCase4(Node* p)
{
    // 违反性质4
    Node* pNode = getParent(p);
    Node*gNode = getGrandParent(p);

    // 第一步
    if (p == pNode->right && pNode == gNode->left)
    {
        rotateLeft(pNode);
        p = p->left;
    }
    else if (p == pNode->left && pNode == gNode->right)
    {
        rotateRight(p);
        p == p->right;
    }
    
    // 第二步
    if (p == pNode->left)
    {
        rotateRight(gNode);
    }
    else
    {
        rotateLeft(gNode);
    }
    pNode->color = RB_BLACK;
    gNode->color = RB_RED;
}


// 先将要删除的结点拷贝一份，删除时，直接删除拷贝的结点
void replaceNode(Node* p, Node* child)
{
    child->parent = p->parent;
    if (p == p->parent->left)
    {
        p->parent->left = child;
    }
    else
    {
        p->parent->right = child;
    }
}

void deleteOneChild(Node* p)
{
    // 要删除的结点至少有一个是非叶子节点
    Node* child = (p->right == nullptr) ? p->left : p->right;
    assert(child);

    replaceNode(p, child);
    if (p->color == RB_BLACK)
    {
        if (child->color == RB_RED)
        {
            child->color = RB_BLACK;
        }
        else
        {
            deleteCase1(child);
        }
    }
    free(p);
}

void deleteCase1(Node* p)
{
    // 结点p 和它的父结点都是黑色结点
    if (p->parent != nullptr)
    {
        deleteCase2(p);
    }
}

void deleteCase2(Node* p)
{
    Node* sNode = getSibling(p);

    if (sNode->color == RB_RED)
    {
        p->parent->color = RB_RED;
        sNode->color = RB_BLACK;

        if (p == p->parent->left)
        {
            rotateLeft(p->parent);
        }
        else
        {
            rotateRight(p->parent);
        }        
    }
    deleteCase3(p);
}

void deleteCase3(Node* p)
{
    Node* sNode = getSibling(p);
    
    if ((p->parent->color == RB_BLACK) && (sNode->color == RB_BLACK) &&
        (sNode->left->color == RB_BLACK) && (sNode->right->color == RB_BLACK))
    {
        sNode->color = RB_RED;
        deleteCase1(p->parent);
    }
    else
    {
        deleteCase4(p);
    }
}

void deleteCase4(Node* p)
{
    Node* sNode = getSibling(p);
    
    if ((p->parent->color == RB_RED) && (sNode->color == RB_BLACK) &&
        (sNode->left->color == RB_BLACK) && (sNode->right->color == RB_BLACK))
    {
        sNode->color = RB_RED;
        p->parent->color = RB_BLACK;
    }
    else
    {
        deleteCase5(p);
    }    
}

void deleteCase5(Node* p)
{
    // 当前结点颜色是黑色，兄弟结点是黑色，兄弟的左子结点是红色，右子结点是黑色
    Node* sNode = getSibling(p);

    if (sNode->color == RB_BLACK) 
    {
        if ((p == p->parent->left) && (sNode->right->color == RB_BLACK) &&
            (sNode->left->color == RB_RED)) 
        {
            sNode->color = RB_RED;  // 变色处理
            sNode->left->color = RB_BLACK;
            rotateRight(sNode);
        } 
        else if ((p == p->parent->right) && (sNode->left->color == RB_BLACK) &&
                 (sNode->right->color == RB_RED)) 
        {
            // 变色处理
            sNode->color = RB_RED;
            sNode->right->color = RB_BLACK;
            rotateLeft(sNode);
        }
    }
    DeleteCase6(p);    
}

void DeleteCase6(Node* p) 
{
    // 当前结点颜色是黑色且为父结点的左子结点，兄弟结点是黑色，
    // 兄弟结点的右子结点是红色，兄弟结点的左子结点颜色是任意色
    Node* sNode = getSibling(p);

    sNode->color = p->parent->color;
    p->parent->color = RB_BLACK;
    if (p == p->parent->left) 
    {
        sNode->right->color = RB_BLACK; // 变色
        rotateLeft(p->parent);   // 旋转
    } 
    else
    {
        sNode->left->color = RB_BLACK;
        rotateRight(p->parent);
    }
}

int main(int argc, char *argv[])
{


    return 0;
}