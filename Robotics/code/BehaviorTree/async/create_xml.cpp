#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/xml_parsing.h>
#include <behaviortree_cpp/json_export.h>
#include <fstream>

struct Pose2D {
    double x;
    double y;
    double theta;
};

void PoseToJson(nlohmann::json& dest, const Pose2D& pose) {
  dest["x"] = pose.x;
  dest["y"] = pose.y;
  dest["theta"] = pose.theta;
}

int main(int argc, char const *argv[])
{
  BT::BehaviorTreeFactory factory;
  
  auto xmls_models = BT::writeTreeNodesModelXML(factory, "my_behavior_tree.xml");
  // std::cout << xmls_models << std::endl;
  std::ofstream file("my_behavior_tree.xml");
  if (file.is_open()) {
      file << xmls_models;
      file.close();
      std::cout << "File written successfully" << std::endl;
  } else {
    std::cerr << "Unable to open file";
    return -1;
  }

  // BT::JsonExporter::get().addConverter<Pose2D>();
  // BT::RegisterJsonDefinition<Pose2D>(PoseToJson);

  return 0;
}
