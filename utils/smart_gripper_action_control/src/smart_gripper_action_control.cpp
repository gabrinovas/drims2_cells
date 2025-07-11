#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "control_msgs/action/gripper_command.hpp"

namespace smart_gripper_action_control
{
class SmartGripperActionControl : public rclcpp::Node
{
public:
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ServerGoalHandle<GripperCommand>;

  SmartGripperActionControl(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("smart_gripper_action_controller", options)
  {
    using namespace std::placeholders;

    // Create the action server
    action_server_ = rclcpp_action::create_server<GripperCommand>(
      this,
      "gripper_cmd",
      std::bind(&SmartGripperActionControl::handle_goal, this, _1, _2),
      std::bind(&SmartGripperActionControl::handle_cancel, this, _1),
      std::bind(&SmartGripperActionControl::handle_accepted, this, _1));

    // Create the service clients
    gripper_in_client_ = this->create_client<std_srvs::srv::Trigger>("grip_in");
    gripper_out_client_ = this->create_client<std_srvs::srv::Trigger>("grip_out");
  }

private:
  rclcpp_action::Server<GripperCommand>::SharedPtr action_server_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_in_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_out_client_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GripperCommand::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: position=%.2f, max_effort=%.2f",
                goal->command.position, goal->command.max_effort);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    std::thread{std::bind(&SmartGripperActionControl::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGripper> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing gripper command");

    auto result = std::make_shared<GripperCommand::Result>();
    const auto goal = goal_handle->get_goal();    
    const bool is_grip_out = goal->command.position > 0.0;

    auto client = is_grip_out ? gripper_out_client_ : gripper_in_client_;
    std::string service_name = is_grip_out ? "gripper_out" : "gripper_in";

    if (!client->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name.c_str());
        goal_handle->abort(result);
        return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);

     if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::FutureReturnCode::SUCCESS || !future.get()->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to call %s", service_name.c_str());
        goal_handle->abort(result);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "%s executed successfully", service_name.c_str());

    result->stalled = false;
    result->reached_goal = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
 }

};  // class SmartGripperActionControl

}  // namespace smart_gripper_action_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_gripper_action_control::SmartGripperActionControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

