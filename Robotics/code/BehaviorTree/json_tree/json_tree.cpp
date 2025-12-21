#include <iostream>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/json_export.h>

class MyCustomAction : public BT::SyncActionNode
{
public:
    MyCustomAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return{};
    }

    virtual BT::NodeStatus tick() override
    {
        std::cout << this->name() << " is ticking" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

int main()
{
    BT::BehaviorTreeFactory factory;

    // 注册自定义节点类型
    factory.registerNodeType<MyCustomAction>("MyCustomAction");

    // 使用 addConverter 加载并注册 JSON 转换器
    BT::JSONFileToBT converter;
    factory.addConverter(BT::ConvertFromString(&converter));

    // 从 JSON 文件加载行为树
    auto tree = factory.createTreeFromFile("my_behavior_tree.json");

    // 执行行为树
    for (int i = 0; i < 10; i++)
    {
        tree.tickRoot();
    }

    return 0;
}