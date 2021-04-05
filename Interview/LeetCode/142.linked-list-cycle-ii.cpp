/*
 * @Author: JohnJeep
 * @Date: 2021-04-03 18:15:55
 * @LastEditTime: 2021-04-05 00:12:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
/*
 * @lc app=leetcode id=142 lang=cpp
 *
 * [142] Linked List Cycle II
 *
 * https://leetcode.com/problems/linked-list-cycle-ii/description/
 *
 * algorithms
 * Medium (39.84%)
 * Likes:    3963
 * Dislikes: 296
 * Total Accepted:    434.4K
 * Total Submissions: 1.1M
 * Testcase Example:  '[3,2,0,-4]\n1'
 *
 * Given a linked list, return the node where the cycle begins. If there is no
 * cycle, return null.
 * 
 * There is a cycle in a linked list if there is some node in the list that can
 * be reached again by continuously following the next pointer. Internally, pos
 * is used to denote the index of the node that tail's next pointer is
 * connected to. Note that pos is not passed as a parameter.
 * 
 * Notice that you should not modify the linked list.
 * 
 * 
 * Example 1:
 * 
 * 
 * Input: head = [3,2,0,-4], pos = 1
 * Output: tail connects to node index 1
 * Explanation: There is a cycle in the linked list, where tail connects to the
 * second node.
 * 
 * 
 * Example 2:
 * 
 * 
 * Input: head = [1,2], pos = 0
 * Output: tail connects to node index 0
 * Explanation: There is a cycle in the linked list, where tail connects to the
 * first node.
 * 
 * 
 * Example 3:
 * 
 * 
 * Input: head = [1], pos = -1
 * Output: no cycle
 * Explanation: There is no cycle in the linked list.
 * 
 * 
 * 
 * Constraints:
 * 
 * 
 * The number of the nodes in the list is in the range [0, 10^4].
 * -10^5 <= Node.val <= 10^5
 * pos is -1 or a valid index in the linked-list.
 * 
 * 
 * 
 * Follow up: Can you solve it using O(1) (i.e. constant) memory?
 * 
 */

// @lc code=start
/**
 * Definition for singly-linked list.
 * struct ListNode {
 *     int val;
 *     ListNode *next;
 *     ListNode(int x) : val(x), next(NULL) {}
 * };
 */
class Solution {
public:
    ListNode *detectCycle(ListNode *head) {
       ListNode* fast = head, *slow = head;
        // 判断时候存在环路
       do {
           if (!fast || !fast->next) {
               return nullptr;
           }
           fast = fast->next->next;
           slow = slow->next;
       } while (fast != slow);

       // 若链表有环，判断环的入口
       fast = head;
       while (fast != slow) {
           slow = slow->next;
           fast = fast->next;
       }
       return fast;
    }
};
// @lc code=end

