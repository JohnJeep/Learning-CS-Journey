/*
 * @Author: your name
 * @Date: 2020-07-26 15:22:48
 * @LastEditTime: 2020-07-26 15:31:46
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Learning-Computer-Journey\Interview\CodingInterview\list.h
 */ 

#ifndef __LISH_H
#define __LISH_H

typedef struct tag_listnode
{
    int value;
    ListNode* next;
}ListNode;

ListNode* CreateListNode(int value);
void ConnectListNodes(ListNode* current, ListNode* next);
void PrintListNode(ListNode* node);
void PrintList(ListNode* head);
void DestroyList(ListNode* head);
void AddToTail(ListNode** head, int value);
void RemoveNode(ListNode** head, int value);

#endif // !__LISH_H