/*
 * @Author: JohnJeep
 * @Date: 2020-07-20 22:44:25
 * @LastEditTime: 2020-08-20 11:06:59
 * @LastEditors: Please set LastEditors
 * @Description: 线性表的链式实现头文件
 */ 
typedef void LinkList;

// 定义节点结构体
typedef struct tag_linkListNode
{
    struct tag_linkListNode *next;
}LinKListNode;

// 定义链表的结构体
typedef struct tag_linkList
{
    int length;
    LinKListNode header;   // 头结点
}S_LinkList;


LinkList* linkListCreate();                                       // 创建链表
void linkListDestory(LinkList* list);                             // 销毁链表
void linkListClear(LinkList* list);                               // 链表恢复到初始状态
int linkListLength(LinkList* list);                               // 获得链表的长度
int linkListInsert(LinkList* list, LinKListNode* node, int pos);  // 向链表中插入节点
LinKListNode* linkListGet(LinkList* list, int pos);               // 获得链表中的元素
LinKListNode* linkListDelete(LinkList* list, int pos);            // 链表节点的删除