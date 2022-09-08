#include "lifecycle_controller.hpp"


using namespace std::chrono_literals;

char key(' ');

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


LifecycleController::LifecycleController(const std::string & node_name)
: Node(node_name)
{
	this->declare_parameter<std::string>("lc_name", "");
	this->declare_parameter<std::string>("get_state_topic", "");
	this->declare_parameter<std::string>("change_state_topic", "");

	this->get_parameter("lc_name", lifecycle_node);
	this->get_parameter("get_state_topic", get_state_topic);
	this->get_parameter("change_state_topic", change_state_topic);

	if (lifecycle_node == ""){
		RCLCPP_WARN(get_logger(), "Please set the lifecycle node argument name while running. Ex: --ros-args -p lc_name:=mmor");
	    std::cout<<"Exiting the node"<<std::endl;
	    rclcpp::shutdown();
	}

	if (get_state_topic == "" && change_state_topic ==""){
		node_get_state_topic = lifecycle_node + "/get_state"; 
	    node_change_state_topic = lifecycle_node + "/change_state";
	}
	else{

		node_get_state_topic = get_state_topic; 
	    node_change_state_topic = change_state_topic;
	}
  
}
  

void LifecycleController::init()
{
	client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
	node_get_state_topic);
	client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
	node_change_state_topic);
}


bool LifecycleController::get_state(std::chrono::seconds time_out = 10s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    
	if (!client_get_state_->wait_for_service(time_out)) {
	RCLCPP_ERROR(
	get_logger(),
	"Service %s is not available.",
	client_get_state_->get_service_name());
		
	RCLCPP_ERROR(
	get_logger(), "Waited for 10 sec,  failed to get current state for node %s", lifecycle_node.c_str());

	return false;
	
	}

	// We send the service request for asking the current
	auto future_result = client_get_state_->async_send_request(request).future.share();
    
	// Let's wait until we have the answer from the node.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	RCLCPP_ERROR(
	get_logger(), "Server time out while getting current state for node %s", lifecycle_node.c_str());
	RCLCPP_ERROR(
	get_logger(), "Waited for 10 sec,  failed to get current state for node %s", lifecycle_node.c_str());
	return false;
	
	}

	// We have an succesful answer. So let's print the current state.
	if (future_result.get()) {
	RCLCPP_INFO(
	get_logger(), "Node %s has current state %s.",
	lifecycle_node.c_str(), future_result.get()->current_state.label.c_str());
	return true;
	} else {
	RCLCPP_ERROR(
	get_logger(), "Waited for 10 sec,  failed to get current state for node %s", lifecycle_node.c_str());
	return false;
	}
}


bool LifecycleController::change_state(std::uint8_t transition, std::chrono::seconds time_out = 10s)
{
	auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
	request->transition.id = transition;

	if (!client_change_state_->wait_for_service(time_out)) {
		return false;
	}

	// We send the request with the transition we want to invoke.
	auto future_result = client_change_state_->async_send_request(request).future.share();

	// Let's wait until we have the answer from the node.
	auto future_status = wait_for_result(future_result, time_out);

	if (future_status != std::future_status::ready) {
	
		return false;
	}

	// We have an answer, let's print our success.
	if (future_result.get()->success) {
	RCLCPP_INFO(
	get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
	return true;
	} else {
	RCLCPP_ERROR(
	get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
	return false;
	}
}


// For non-blocking keyboard inputs
// Source: ROS teleop_twist_keyboard package
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


const char* display = R"(
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
 * default lifecycle of a node based on keyborad inputs.
 */
void callee_script(std::shared_ptr<LifecycleController> lifecycle_controller)
{

	std::cout<<display<<std::endl;
	int lc_state = lifecycle_controller->get_state();
	
	while(lc_state){
    std::cout<<"Enter the key or press T to terminate and exit :"<<std::endl;
	key = getch();

    // UNCONFIGURED -->  Configuring   --> INACTIVE
    if (key == 'C'){

	std::cout<<"configure"<<std::endl;
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
	lc_state=lifecycle_controller->get_state();
	}

	// INACTIVE     -->  Activating    --> ACTIVE
	if (key == 'A'){
	
	std::cout<<"activate"<<std::endl;
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
	lc_state=lifecycle_controller->get_state();

	}

	// ACTIVE       -->  Configuring   --> INACTIVE
	if (key == 'D'){

	std::cout<<"deactivate"<<std::endl;
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
	lc_state=lifecycle_controller->get_state();
	
	}

	// INACTIVE     -->  CleaningUp    --> UNCONFIGURED
	if (key == 'R'){
	
	std::cout<<"cleanup"<<std::endl;  
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
	lc_state=lifecycle_controller->get_state();	

	}
	
	// INACTIVE     -->  ShuttingDown  --> FINALIZED
	if (key == 'W'){

	std::cout<<"inactive shutdown"<<std::endl;  
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
	lc_state=lifecycle_controller->get_state();	

	}

	// ACTIVE       -->  ShuttingDown  --> FINALIZED
	if (key == 'X'){
	
	std::cout<<"active shutdown"<<std::endl;  
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
	lc_state=lifecycle_controller->get_state();	

	}

	// UNCONFIGURED -->  ShuttingDown  --> FINALIZED
	if (key == 'S'){
	
	std::cout<<"unconfig shutdown"<<std::endl;
	lc_state=lifecycle_controller->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
	lc_state=lifecycle_controller->get_state();
	
	}

	// Terminate
	if (key == 'T'){
	break;
	}

	}

	std::cout<<"Exiting the node"<<std::endl;
	rclcpp::shutdown();

}


int main(int argc, char ** argv)
{
	// force flush of the stdout buffer.
	// this ensures a correct sync of all prints
	// even when executed simultaneously within the launch file.
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);

	rclcpp::init(argc, argv);

	auto lifecycle_controller = std::make_shared<LifecycleController>("lifecycle_controller");
	lifecycle_controller->init();
	
	rclcpp::executors::SingleThreadedExecutor exe;
	exe.add_node(lifecycle_controller);


	std::shared_future<void> script = std::async(
	std::launch::async,
	std::bind(callee_script, lifecycle_controller));
	exe.spin_until_future_complete(script);

	rclcpp::shutdown();
	
	return 0;
}

