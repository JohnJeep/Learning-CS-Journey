<!--
 * @Author: JohnJeep
 * @Date: 2021-01-11 22:23:18
 * @LastEditTime: 2021-09-23 18:44:22
 * @LastEditors: Windows10
 * @Description: In User Settings Edit
-->

#  第5题：用两个栈实现队列

# 题目描述
用两个栈来实现一个队列，完成队列的 Push 和 Pop 操作。 队列中的元素为 `int` 类型。

# 解题思路
- stack 的特点：先进后出
- queue 的特点：先进先出
- push 操作就直接往 stack1 中 push， pop 操作需要分类一下：如果 stack2 为空，那么需要将 stack1 中的数据转移到 stack2 中，然后在对 stack2 进行 pop，如果 stack2 不为空，直接 pop 就 ok。

具体代码实现
```cpp
class Solution
{
public:
    void push(int node) {
        stack1.push(node);
    }

    int pop() {
        if (stack2.empty()) {
            while(!stack1.empty()){
                stack2.push(stack1.top());
                stack1.pop();
            }
        }
        int ret = stack2.top();
        stack2.pop();
        return ret;
    }

private:
    stack<int> stack1;
    stack<int> stack2;
};
```

# 复杂度
- push时间复杂度：$\Omicron(1)$
- pop空间复杂度：$\Omicron(1)$


> 所有题目按照牛客网的顺序来编写，而非剑指offer书上的顺序。所有题目都采用C++实现的，都能在牛客网的OJ上面直接运行。