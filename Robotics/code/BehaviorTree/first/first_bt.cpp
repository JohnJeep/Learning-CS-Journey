#include <behaviortree_cpp/bt_factory.h>

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
  {}
  ~ApproachObject() override = default; // 现代C++写法，表示使用基类的析构函数，显式默认析构函数

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override {
    std::cout << "Hello, Behavior Tree, name: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

BT::NodeStatus CheckBattery()
{
  std::cout << "Checking battery status..." << std::endl;
  return BT::NodeStatus::SUCCESS;
}

class GripperInterface
{
public:
  GripperInterface() : is_open_(true) 
    {}
  ~GripperInterface() {}

  BT::NodeStatus open()
  {
    is_open_ = true;
    std::cout << "Gripper opened." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus close()
  {
    is_open_ = false;
    std::cout << "Gripper closed." << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool is_open_; // share information about gripper state
};

int main(int argc, char const *argv[])
{
  BT::BehaviorTreeFactory factory;
  
  // 注册自定义节点
  factory.registerNodeType<ApproachObject>("ApproachObject");

  // 注册简单条件节点
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

  GripperInterface gripper;

  // 注册简单动作节点
  factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));

  // 从XML文件创建行为树 
  auto tree = factory.createTreeFromFile("../my_tree.xml");
  
  // 单次次tick行为树，返回执行后的根节点状态
  // tree.tickRoot(); // BT 3.8
  // tree.tickOnce();

  // 循环执行行为树，直到根节点返回 SUCCESS 或 FAILURE
  BT::NodeStatus status = tree.tickWhileRunning();
  std::cout << "Final Status: " << BT::toStr(status) << std::endl;

  return 0;
}
