#include <iostream>
#include <behaviortree_cpp_v3/bt_factory.h>

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
  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));
  factory.registerSimpleAction("CloseGripper", std::bind(&GripperInterface::close, &gripper));
  auto tree = factory.createTreeFromFile("../my_tree.xml");
  // tree.tickRoot();
  tree.tickRootWhileRunning();

  return 0;
}
