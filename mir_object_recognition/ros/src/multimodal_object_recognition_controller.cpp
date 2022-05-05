#include "mir_object_recognition/multimodal_object_recognition_controller.hpp"


using namespace std::chrono_literals;

key(' ');

// which node to handle
lifecycle_node = "lc_talker";  //lc_pubsub 

// Every lifecycle node has various services
// attached to it. By convention, we use the format of
// <node name>/<service name>.
// In this demo, we use get_state and change_state
// and thus the two service topics are:
// lc_pubsub/get_state
// lc_pubsub/change_state 
node_get_state_topic = "lc_talker/get_state"; //lc_pubsub 
node_change_state_topic = "lc_talker/change_state";  //lc_pubsub 

template<typename FutureT, typename WaitTimeT>
std::future_status wait_for_result( FutureT & future, WaitTimeT time_to_wait)
{
	auto end = std::chrono::steady_clock::now() + time_to_wait;
	std::chrono::milliseconds wait_period(100);
	std::future_status status = std::future_status::timeout;
	do {
	auto now = std::chrono::steady_clock::now();
	auto time_left = end - now;
	if (time_left <= std::chrono::seconds(0)) {break;}
	status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
	} while (rclcpp::ok() && status != std::future_status::ready);
	return status;
}





explicit MultiModalObjectRecognitionController::MultiModalObjectRecognitionController(const std::string & node_name)
: Node(node_name)
{

}
  

void MultiModalObjectRecognitionController::init()
{
	// Every lifecycle node spawns automatically a couple
	// of services which allow an external interaction with
	// these nodes.
	// The two main important ones are GetState and ChangeState.
	client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
	node_get_state_topic);
	client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
	node_change_state_topic);
}



  /// Requests the current state of the node
  /**
   * In this function, we send a service request
   * asking for the current state of the node
   * lc_talker.
   * If it does return within the given time_out,
   * we return the current state of the node, if
   * not, we return an unknown state.
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
 
void MultiModalObjectRecognitionController::get_state(std::chrono::seconds time_out = 3s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

	if (!client_get_state_->wait_for_service(time_out)) {
	RCLCPP_ERROR(
	get_logger(),
	"Service %s is not available.",
	client_get_state_->get_service_name());
	//return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}

	// We send the service request for asking the current
	// state of the lc_talker node.
	auto future_result = client_get_state_->async_send_request(request);

	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	RCLCPP_ERROR(
	get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
	//return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}

	// We have an succesful answer. So let's print the current state.
	if (future_result.get()) {
	RCLCPP_INFO(
	get_logger(), "Node %s has current state %s.",
	lifecycle_node, future_result.get()->current_state.label.c_str());
	//return future_result.get()->current_state.id;
	} else {
	RCLCPP_ERROR(
	get_logger(), "Failed to get current state for node %s", lifecycle_node);
	//return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}
}

  /// Invokes a transition
  /**
   * We send a Service request and indicate
   * that we want to invoke transition with
   * the id "transition".
   * By default, these transitions are
   * - configure
   * - activate
   * - cleanup
   * - shutdown
   * \param transition id specifying which
   * transition to invoke
   * \param time_out Duration in seconds specifying
   * how long we wait for a response before returning
   * unknown state
   */
void MultiModalObjectRecognitionController::change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
	request->transition.id = transition;

	if (!client_change_state_->wait_for_service(time_out)) {
	RCLCPP_ERROR(
	get_logger(),
	"Service %s is not available.",
	client_change_state_->get_service_name());
	//return false;
	}

	// We send the request with the transition we want to invoke.
	auto future_result = client_change_state_->async_send_request(request);

	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	RCLCPP_ERROR(
	get_logger(), "Server time out while getting current state for node %s", lifecycle_node);
	//return false;
	}

	// We have an answer, let's print our success.
	if (future_result.get()->success) {
	RCLCPP_INFO(
	get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
	//return true;
	} else {
	RCLCPP_WARN(
	get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
	//return false;
	}
}






// For non-blocking keyboard inputs
int getch(void)
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 1;
	newt.c_cc[VTIME] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}


msg = R"(
Reading from the keyboard and changing states!
########################################################################
Key | Current State --> Via (Intermediate state) --> Destination State
------------------------------------------------------------------------
C:  | UNCONFIGURED -->  Configuring   --> INACTIVE
S:  | UNCONFIGURED -->  ShuttingDown  --> FINALIZED
A:  | INACTIVE     -->  Activating    --> ACTIVE
R:  | INACTIVE     -->  CleaningUp    --> UNCONFIGURED
W:  | INACTIVE     -->  ShuttingDown  --> FINALIZED
D:  | ACTIVE       -->  Configuring   --> INACTIVE
X:  | ACTIVE       -->  ShuttingDown  --> FINALIZED

########################################################################

Tip: Initial state is UNCONFIGURED state if the lifecycle node is ready. 
Press "C" making the node state go from UNCONFIGURED --> INACTIVE
)";



/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void callee_script(std::shared_ptr<MultiModalObjectRecognitionController> lc_client)
{

	std::cout<<msg<<std::endl;
	while(rclcpp::ok()){
	key = getch();


	if (key == 'C'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"configure"<<std::endl;
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	lc_client->get_state();

	}
	}


	// activate
	if (key == 'A'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"activate"<<std::endl;
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
	lc_client->get_state();

	}
	}
	// deactivate
	if (key == 'D'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"deactivate"<<std::endl;
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
	lc_client->get_state();
	}

	}

	// we cleanup
	if (key == 'R'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"cleanup"<<std::endl;  
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
	lc_client->get_state();
	}

	}

	if (key == 'W'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"inactive shutdown"<<std::endl;  
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
	lc_client->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  
	}

	}

	if (key == 'X'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"active shutdown"<<std::endl;  
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
	lc_client->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  

	}

	}

	// and finally shutdown
	// Note: We have to be precise here on which shutdown transition id to call
	// We are currently in the unconfigured state and thus have to call
	// TRANSITION_UNCONFIGURED_SHUTDOWN
	if (key == 'S'){
	//time_between_state_changes.sleep();
	if (rclcpp::ok()) {
	std::cout<<"unconfig shutdown"<<std::endl;
	lc_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
	lc_client->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  

	}
	}


	if (key == 'T'){
	break;
	}

	}

	std::cout<<"Press Cntr+C to exit"<<std::endl;

}




int main(int argc, char ** argv)
{
	// force flush of the stdout buffer.
	// this ensures a correct sync of all prints
	// even when executed simultaneously within the launch file.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	auto mmor_controller = std::make_shared<MultiModalObjectRecognitionController>("mmor_controller");
	mmor_controller->init();
	//rclcpp::WallRate time_between_state_changes(0.1);
	rclcpp::executors::SingleThreadedExecutor exe;
	exe.add_node(mmor_controller);


	std::shared_future<void> script = std::async(
	std::launch::async,
	std::bind(callee_script, mmor_controller));
	exe.spin_until_future_complete(script);

	rclcpp::shutdown();



	return 0;
}

