#include <behaviortree_cpp/behavior_tree.h>

struct Pose2D
{
  double x, y, theta;
};

class MoveBaseAction : public BT::StatefulActionNode
{
  public:
    MoveBaseAction(const std::string& name, const BT::NodeConfig& config)
      : StatefulActionNode(name, config)  
    {}
    ~MoveBaseAction() {}

    static BT::PortsList providedPorts()
    {
      return { BT::InputPort<Pose2D>("goal") };
    }

    BT::NodeStatus onStart() override
    {

    }

    BT::NodeStatus onRunning() override
    {

    }

    void onHalted() override
    {
    }
  private:
};