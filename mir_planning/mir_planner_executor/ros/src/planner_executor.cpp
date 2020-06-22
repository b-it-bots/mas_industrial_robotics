/*
 * Copyright [2017] <Bonn-Rhein-Sieg University>
 *
 * Author: Torsten Jandt
 *
 */

#include <mir_planner_executor/planner_executor.h>
#include <ros/console.h>

#include <mir_planner_executor/actions/insert/combined_insert_action.h>
#include <mir_planner_executor/actions/move/move_action.h>
#include <mir_planner_executor/actions/perceive/combined_perceive_action.h>
#include <mir_planner_executor/actions/pick/combined_pick_action.h>
#include <mir_planner_executor/actions/place/place_action.h>
#include <mir_planner_executor/actions/stage/stage_action.h>
#include <mir_planner_executor/actions/unstage/unstage_action.h>

//#include <mir_audio_receiver/AudioMessage.h>

PlannerExecutor::PlannerExecutor(ros::NodeHandle &nh)
    : server_(nh, "execute_plan", false) {
  server_.registerGoalCallback(
      boost::bind(&PlannerExecutor::executeCallback, this));
  server_.start();

  ros::NodeHandle private_nh("~");
  knowledge_updater_ = new KnowledgeUpdater(nh);

  // audio_publisher_ =
  // private_nh.advertise<mir_audio_receiver::AudioMessage>("/mir_audio_receiver/tts_request",
  // 1);

  addActionExecutor("PICK", new CombinedPickAction());
  addActionExecutor("PLACE", new PlaceAction());
  addActionExecutor("STAGE", new StageAction());
  addActionExecutor("UNSTAGE", new UnstageAction());
  addActionExecutor("MOVE_BASE", new MoveAction());
  addActionExecutor("INSERT", new CombinedInsertAction());
  addActionExecutor("PERCEIVE", new CombinedPerceiveAction());
  ROS_INFO("Node initialized. PlannerExecutor is available!!!");
}

PlannerExecutor::~PlannerExecutor() {}

void PlannerExecutor::addActionExecutor(std::string name,
                                        BaseExecutorAction *action) {
  actions_[toUpper(name)] = action;
  action->initialize(knowledge_updater_);
}

void PlannerExecutor::executeCallback() {
  const mir_planning_msgs::ExecutePlanGoalConstPtr &msg_goal =
      server_.acceptNewGoal();
  const rosplan_dispatch_msgs::CompletePlan &plan = msg_goal->plan;
  const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actions = plan.plan;
  ROS_INFO("Got plan, %d actions", (unsigned int)actions.size());

  bool result = checkPlan(plan);
  if (!result) {
    ROS_ERROR("Plan check failed. Execution fails");
    mir_planning_msgs::ExecutePlanResult msg_result;
    msg_result.success = false;
    server_.setAborted(msg_result);
    return;
  }
  ROS_INFO("Plan seems to be valid");

  int num_of_actions = actions.size();
  for (int i = 0; i < num_of_actions; i++) {
    auto const &action = actions[i];

    if (server_.isPreemptRequested()) {
      ROS_WARN("Preemption is requested. Stopping execution.");
      mir_planning_msgs::ExecutePlanResult msg_result;
      msg_result.success = false;
      server_.setPreempted(msg_result);
      return;
    }

    std::vector<diagnostic_msgs::KeyValue> params = action.parameters;

    /* add info about next action */
    if (i + 1 < num_of_actions) {
      std::string next_action_name = actions[i + 1].name;
      diagnostic_msgs::KeyValue next_action;
      next_action.key = "next_action";
      next_action.value = next_action_name;
      params.push_back(next_action);
    }

    std::string action_name = toUpper(action.name);
    announceAction(action_name, params);

    BaseExecutorAction *executor = getActionExecutor(action_name);
    bool res = executor->execute(action_name, params);

    if (!res) {
      ROS_WARN("\nAction \"%s\" failed, abort plan execution\n",
               action_name.c_str());
      mir_planning_msgs::ExecutePlanResult msg_result;
      msg_result.success = false;
      server_.setAborted(msg_result);
      return;
    }
    ROS_INFO("Finished action \"%s\"\n", action_name.c_str());
  }
  ROS_INFO("Plan execution finished");
  mir_planning_msgs::ExecutePlanResult msg_result;
  msg_result.success = true;
  server_.setSucceeded(msg_result);
}

bool PlannerExecutor::checkPlan(
    const rosplan_dispatch_msgs::CompletePlan &plan) {
  const std::vector<rosplan_dispatch_msgs::ActionDispatch> &actions = plan.plan;
  for (auto const &action : actions) {
    std::string action_name = toUpper(action.name);
    if (!actions_.count(action_name)) {
      ROS_ERROR(
          "Failed to find action executor with name \"%s\" required by "
          "the plan.",
          action_name.c_str());
      return false;
    }
  }
  return true;
}

void PlannerExecutor::announceAction(
    std::string action_name, std::vector<diagnostic_msgs::KeyValue> params) {
  /* announce action with text */
  std::cout << std::endl << std::endl << std::endl << std::endl << std::endl;
  ROS_INFO("Executing action \"%s\"", action_name.c_str());
  ROS_INFO("Parameters :");
  for (diagnostic_msgs::KeyValue kw : params) {
    ROS_INFO_STREAM("    " << kw.key << ": " << kw.value);
  }

  /* announce action with audio */
  /* mir_audio_receiver::AudioMessage audio_msg;
  std::string action_name_string(action_name.c_str());
  std::replace(action_name_string.begin(), action_name_string.end(), '_', ' ');
  std::string audio_message_string = "Executing action " + action_name_string;
  audio_msg.message = audio_message_string;
  audio_publisher_.publish(audio_msg); */
}

std::string PlannerExecutor::toUpper(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(), ::toupper);
  return str;
}

BaseExecutorAction *PlannerExecutor::getActionExecutor(std::string &name) {
  return actions_[toUpper(name)];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "planner_executor");

  ros::NodeHandle nh;

  PlannerExecutor planner_executor(nh);

  ros::spin();
  return 0;
}
