/*
 * @Author: JohnJeep
 * @Date: 2020-07-26 15:22:48
 * @LastEditTime: 2020-07-26 15:31:46
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /linkList.h
 */ 

#ifndef __LINKLIST_H
#define __LINKLIST_H

typedef struct tag_listnode
{
    int value;
    tag_listnode* next;
}ListNode;

ListNode* CreateListNode(int value);
void ConnectListNodes(ListNode* current, ListNode* next);
void PrintListNode(ListNode* node);
void PrintList(ListNode* head);
void DestroyList(ListNode* head);
void AddToTail(ListNode** head, int value);
void RemoveNode(ListNode** head, int value);

#endif 