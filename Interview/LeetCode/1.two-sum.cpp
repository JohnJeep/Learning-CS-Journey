/*
 * @Author: JohnJeep
 * @Date: 2021-03-20 00:03:46
 * @LastEditTime: 2021-04-23 23:33:00
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 */
/*
 * @lc app=leetcode id=1 lang=cpp
 *
 * [1] Two Sum
 */

// @lc code=start
class Solution {
public:
    vector<int> twoSum(vector<int>& nums, int target) {
        // 法一：暴力解法
        // int len = nums.size();
        // for (int i = 0; i < len; i++) {
        //     for (int j = i + 1; j < len; j++) {
        //         if (nums[i] + nums[j] == target) {
        //             return {i, j};
        //         }
        //     }
        // }
        // return {};

        // 利用hashtable实现
        unordered_map<int, int> mp;
        vector<int>result;
        for (int i = 0; i < nums.size(); i++) {
            // mp[nums[i]] = i;
            auto it = mp.find(target - nums[i]);
            if (it != mp.end()) {
                result = {{it->second, i}}; 
            }
            mp[nums[i]] = i; 
        }
        return result;
    }
};
// @lc code=end

