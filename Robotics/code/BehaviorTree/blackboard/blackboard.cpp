
#include "behaviortree_cpp/bt_factory.h"

class SaySomething : public BT::SyncActionNode
{
public:
  SaySomething(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
  {}
  ~SaySomething() {}

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }

  // override the virtual function tick()
  BT::NodeStatus tick() override {
    // BT::Optional<std::string> msg = getInput<std::string>("message"); // BT 3.8
    // read the value from the input port "message"
    BT::Expected<std::string> msg = getInput<std::string>("message");
    if (!msg) {
      throw BT::RuntimeError("missing required input [message]: ", msg.error());
  }

  // use the method value() to extract the valid message.
  std::cout << "SaySomething: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  } 
};

// uses an output port to write a string into an entry.
class ThinkWhatToSay : public BT::SyncActionNode
{
public:
  ThinkWhatToSay(const std::string& name, const BT::NodeConfiguration& config) 
    : BT::SyncActionNode(name, config)
    {}
  ~ThinkWhatToSay() {}

  // return std::unordered_map<std::string, BT::PortInfo>
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("text") };
  }

  // override the virtual function tick()
  // This method will write into the output port "text"
  BT::NodeStatus tick() override {
    std::string content = "I am a bot";
    BT::TreeNode::setOutput("text", content);
    std::cout << "ThinkWhatToSay: " << content << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

};

int main(int argc, char const *argv[])
{
  // create the BehaviorTreeFactory
  BT::BehaviorTreeFactory factory;

  // register the custom nodes into the factory
  factory.registerNodeType<SaySomething>("SaySomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  // read the tree from an XML file
  auto tree = factory.createTreeFromFile("../blackboard.xml");

  // tree.tickRootWhileRunning(); // BT 3.8
  // tick the root until it returns SUCCESS or FAILURE
  tree.tickWhileRunning();

  return 0;
}
