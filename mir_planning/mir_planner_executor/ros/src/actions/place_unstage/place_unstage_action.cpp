#include <mir_planner_executor/actions/place_unstage/place_unstage_action.h>
#include <utility>

PlaceUnstageAction::PlaceUnstageAction() : ExecutorAction("/place_unstage_object_server")
{
  // client_.waitForServer();
}

void PlaceUnstageAction::update_knowledge_base(bool success,
                                               std::vector<diagnostic_msgs::KeyValue> &params)
{
  std::string robot = getValueOf(params, "robot_name");
  std::string location = getValueOf(params, "location");
  std::string object = getValueOf(params, "object");
  std::string platform = getValueOf(params, "platform");
  if (success) {
    knowledge_updater_->remKnowledge("stored",{{"rp",platform},{"o",object}});
    knowledge_updater_->remKnowledge("occupied",{{"rp",platform}});
    knowledge_updater_->addKnowledge("on", {{"o", object}, {"l", location}});
    knowledge_updater_->addKnowledge("perceived", {{"l", location}});
    knowledge_updater_->addKnowledge("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remGoal("holding", {{"r", robot}, {"o", object}});
    knowledge_updater_->remGoal("on", {{"o", object}, {"l", location}});
    knowledge_updater_->remGoal("gripper_is_free", {{"r", robot}});
    knowledge_updater_->remGoal("perceived", {{"l", location}});
  }  else {
    return;
  }
}

void PlaceUnstageAction::updateParamsBasedOnContext(std::vector<diagnostic_msgs::KeyValue> &params)
{
  int location_index = getIndexOf(params, "param_1");
  int object_index = getIndexOf(params, "param_2");
  int platform_index = getIndexOf(params, "param_3");

  if (platform_index > 0) {
    params[platform_index].key = "platform";
  }
  if (object_index > 0) {
    params[object_index].key = "object";
  }
  if (location_index > 0) {
    params[location_index].key = "location";
  }
}

