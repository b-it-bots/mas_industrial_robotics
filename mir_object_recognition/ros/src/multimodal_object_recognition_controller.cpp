#include "multimodal_object_recognition_controller.hpp"


using namespace std::chrono_literals;

char key(' ');


static constexpr char const * lifecycle_node = "lc_talker";
static constexpr char const * node_get_state_topic = "lc_talker/get_state";
static constexpr char const * node_change_state_topic = "lc_talker/change_state";


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





MultiModalObjectRecognitionController::MultiModalObjectRecognitionController(const std::string & node_name)
: Node(node_name)
{
  //this->declare_parameter<std::string>("lifecycle_node_name", "lc_talker");
  //this->get_parameter("lifecycle_node_name", lifecycle_node);
  //node_get_state_topic = lifecycle_node + "/get_state"; 
  //node_change_state_topic = lifecycle_node + "/change_state";  
  
}
  

void MultiModalObjectRecognitionController::init()
{

	client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
	node_get_state_topic);
	client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
	node_change_state_topic);
}


 
void MultiModalObjectRecognitionController::get_state(std::chrono::seconds time_out = 3s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    
	if (!client_get_state_->wait_for_service(time_out)) {
	RCLCPP_WARN(
	get_logger(),
	"Service %s is not available.",
	client_get_state_->get_service_name());
	//return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	
	}

	// We send the service request for asking the current
	// state of the lc_talker node.
	auto future_result = client_get_state_->async_send_request(request).future.share();
    
	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	RCLCPP_WARN(
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
	RCLCPP_WARN(
	get_logger(), "Failed to get current state for node %s", lifecycle_node);
	//return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
	}
}


void MultiModalObjectRecognitionController::change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
	request->transition.id = transition;

	if (!client_change_state_->wait_for_service(time_out)) {
	RCLCPP_WARN(
	get_logger(),
	"Service %s is not available.",
	client_change_state_->get_service_name());
	//return false;
	}

	// We send the request with the transition we want to invoke.
	auto future_result = client_change_state_->async_send_request(request).future.share();

	// Let's wait until we have the answer from the node.
	// If the request times out, we return an unknown state.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	RCLCPP_WARN(
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


const char* msg = R"(
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
)";



/**
 * This is a little independent
 * script which triggers the
 * default lifecycle of a node.
 * It starts with configure, activate,
 * deactivate, activate, deactivate,
 * cleanup and finally shutdown
 */
void callee_script(std::shared_ptr<MultiModalObjectRecognitionController> mmor_controller)
{

	std::cout<<msg<<std::endl;
	while(rclcpp::ok()){
	key = getch();


	if (key == 'C'){
	//time_between_state_changes.sleep();
	
	std::cout<<"configure"<<std::endl;
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	mmor_controller->get_state();
	}


	// activate
	if (key == 'A'){
	//time_between_state_changes.sleep();
	
	std::cout<<"activate"<<std::endl;
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
	mmor_controller->get_state();
	}
	// deactivate
	if (key == 'D'){
	//time_between_state_changes.sleep();

	std::cout<<"deactivate"<<std::endl;
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
	mmor_controller->get_state();


	}

	// we cleanup
	if (key == 'R'){
	//time_between_state_changes.sleep();
	
	std::cout<<"cleanup"<<std::endl;  
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
	mmor_controller->get_state();
	

	}

	if (key == 'W'){
	//time_between_state_changes.sleep();

	std::cout<<"inactive shutdown"<<std::endl;  
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
	mmor_controller->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  
	

	}

	if (key == 'X'){
	//time_between_state_changes.sleep();
	
	std::cout<<"active shutdown"<<std::endl;  
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
	mmor_controller->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  

	

	}

	// and finally shutdown
	// Note: We have to be precise here on which shutdown transition id to call
	// We are currently in the unconfigured state and thus have to call
	// TRANSITION_UNCONFIGURED_SHUTDOWN
	if (key == 'S'){
	//time_between_state_changes.sleep();

	std::cout<<"unconfig shutdown"<<std::endl;
	mmor_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
	mmor_controller->get_state();
	std::cout<<"Press T to terminate"<<std::endl;  

	
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

