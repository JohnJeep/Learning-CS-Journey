## BehaviorTree

**定义**：

**行为树** 是一种用于建模AI决策逻辑的**图形化编程架构**。它通过**节点**和**树状结构**来控制任务流程，广泛应用于游戏AI、机器人、自动驾驶等领域，以实现复杂、响应式且易于调试的智能行



**特点**：

行为树是树状的结构，它的逻辑流程是由`xml`文件描述的。行为树本身并不具体实现机器人的执行内容，它只负责将执行内容进行编排。



优点

1. 调试方便，机器行为的变化都可以追溯。机器行为的变化可记录回放。行为变化也可实时监控。
2. 代码复用率高。不同的功能只需少量代码的修改和机器行为的重新组织。



### Nodes

任意的 TreeNode 都可以看成是一种 callback 机制。

LeafNodes

- ActionNode
- ConditionNode





### 开源库

`BehaviorTree.CPP`是一个开源的`C++`行为树库。



### References

1. BehaviorTree.CPP: https://www.behaviortree.dev/
2. 知乎：ROS2中的行为树 BehaviorTree：https://zhuanlan.zhihu.com/p/534072049

