#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>

struct Position2D
{
  double x;
  double y;
};

// template specialization to convert Position2D from string
template<> 
inline Position2D BT::convertFromString(BT::StringView str) {
  auto parts = BT::splitString(str, ';');
  if (parts.size() != 2) {
    throw BT::RuntimeError("invalid input");
  } else {
    Position2D p2d;
    p2d.x = BT::convertFromString<double>(parts[0]);
    p2d.y = BT::convertFromString<double>(parts[1]);
    return p2d;
  }
}

class CalGoal : public BT::SyncActionNode
{
public:
  CalGoal(const std::string& name, const BT::NodeConfig& config) 
    : BT::SyncActionNode(name, config)  
  {}
  ~CalGoal() {}

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<Position2D>("goal") };
  }

  BT::NodeStatus tick() override {
    Position2D mygoal = {1.1, 2.5};
    setOutput<Position2D>("goal", mygoal);
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintTarget : public BT::SyncActionNode
{
public:
  PrintTarget(const std::string& name, const BT::NodeConfig& config) 
    : BT::SyncActionNode(name, config)  
  {}
  ~PrintTarget() {}

  static BT::PortsList providedPorts() {
    const char*  desc = "simple print";
    return { BT::InputPort<Position2D>("target", desc) };
  }

  BT::NodeStatus tick() override {
    auto result = getInput<Position2D>("target");
    if (!result) {
      throw BT::RuntimeError("Error reading port [target]:", result.error());
    }

    Position2D target = result.value();
    std::cout << "Target position: (" << target.x << ", " << target.y << ")\n";
    return BT::NodeStatus::SUCCESS;
  }
};


// static const char* xml_text = R"(
//  <root BTCPP_format="4" >
//      <BehaviorTree ID="MainTree">
//         <Sequence name="root">
//             <CalGoal goal="{GoalPosition}" />
//             <PrintTarget   target="{GoalPosition}" />
//             <Script        code=" OtherGoal:='-1;3' " />
//             <PrintTarget   target="{OtherGoal}" />
//         </Sequence>
//      </BehaviorTree>
//  </root>
//  )";

int main(int argc, char const *argv[])
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<CalGoal>("CalGoal");
  factory.registerNodeType<PrintTarget>("PrintTarget");

  // auto tree = factory.createTreeFromText(xml_text);
  auto tree = factory.createTreeFromFile("../goal.xml");
  tree.tickWhileRunning();

  return 0;
}
