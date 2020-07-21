/*
 * @Author: JohnJeep
 * @Date: 2020-07-21 21:25:28
 * @LastEditTime: 2020-07-21 21:27:17
 * @LastEditors: Please set LastEditors
 * @Description: 循环链表的底层接口
 * @FilePath: /03_CircularCircularList.h
 */ 
typedef void CircularList;

// 定义节点结构体
typedef struct tag_CircularListNode
{
    struct tag_CircularListNode *next;
}CircularListNode;

// 定义链表的结构体
typedef struct tag_CircularList
{
    int length;
    CircularListNode header;   // 头结点
    CircularListNode* slider;  // 游标
}C_CircularList;


CircularList* CircularListCreate();                                           // 创建链表
int CircularListLength(CircularList* list);                                   // 获得链表的长度
int CircularListInsert(CircularList* list, CircularListNode* node, int pos);  // 向链表中插入节点
void CircularListDestory(CircularList* list);                                 // 销毁链表
void CircularListClear(CircularList* list);                                   // 链表恢复到初始状态
CircularListNode* CircularListGet(CircularList* list, int pos);               // 获得链表中的元素
CircularListNode* CircularListDelete(CircularList* list, int pos);            // 链表节点的删除
CircularListNode* CircularListReset(CircularList* list);                      // 让游标重新指向链表的头部
CircularListNode* CircularListCurrent(CircularList* list);                    // 获取游标所指向的位置
CircularListNode* CircularListNext(CircularList* list);                       // 下一个链表的节点
CircularListNode* CircularListDeleteNode(CircularList* list, CircularListNode* node);
