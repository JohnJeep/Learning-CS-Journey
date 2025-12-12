
#include "behaviortree_cpp/bt_factory.h"

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
  {}
  ~SaySomething() {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("message")};
  }

  BT::NodeStatus tick() override {
    // BT::Optional<std::string> msg = getInput<std::string>("message"); // BT 3.8
    BT::Expected<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  std::cout << "SaySomething: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  } 
};

class ThinkWhatToSay : public BT::SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {}
  ~ThinkWhatToSay() {}

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("text") };
  }

  BT::NodeStatus tick() override {
    std::string answer = "The answer is 42.";
    setOutput("text", answer);
    std::cout << "ThinkWhatToSay: " << answer << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

};

int main(int argc, char const *argv[])
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  auto tree = factory.createTreeFromFile("../blackboard.xml");
  // tree.tickRootWhileRunning(); // BT 3.8
  tree.tickWhileRunning();

  return 0;
}
