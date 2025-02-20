#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/Float64.h"
#include <vector>
#include "std_srvs/Empty.h"
#include "inverted_pendulum_pkg/CandidateSolution.h"
#include "gazebo_msgs/LinkStates.h"
#include <thread>
#include <mutex>
#include <atomic>

namespace ipp = inverted_pendulum_pkg;

// Global variables 
double Kp_c, Ki_c, Kd_c, Kp_p, Ki_p, Kd_p;
double yaw_rate  = 0, current_pos = 0;
double yaw_loss = 0, pos_loss = 0;
ros::ServiceClient reset_simulation_client;
std_srvs::Empty reset_simulation_srv;
ros::Publisher cmd_pub;
bool break_control_loop = false;


// debug variables 
int num_available_callbacks = 0;
int control_call_count      = 0; 
int update_call_count       = 0;


// thread safety 
std::mutex control_vars_mutex;
std::atomic<bool> start_control{false};
ros::CallbackQueue state_update_queue;
std::thread control_thread;


void state_update_callback(const gazebo_msgs::LinkStatesConstPtr& state_msg){

    ++update_call_count;
    ROS_INFO_THROTTLE(1,"[update]: %d", update_call_count);

    // get cart-pole pose data
    static std::vector<std::string> link_names_ = state_msg->name;

    int index_cart = -1, index_pole = -1; // FUTURE: The implementation of this loop could be more efficient by searching for both links in one loop 
    for (int i = 0; i < link_names_.size(); ++i){
        if (link_names_[i] == "cart_pole::cart_link"){
            index_cart = i;
            break;
        }
    }

    for (int i = 0; i < link_names_.size(); ++i){
        if (link_names_[i] == "cart_pole::pole_link"){
            index_pole = i;
            break;
        }
    }

    // update control variables
    if (index_cart == -1 || index_pole == -1){
        ROS_FATAL("[tuned controller]: could not find cart or pole link. Exiting...");
    }
    else {
        std::lock_guard<std::mutex> lock(control_vars_mutex); // prevent race conditions 
        current_pos = state_msg->pose[index_cart].position.x; 
        yaw_rate = state_msg->twist[index_pole].angular.y;
    }
    
    return;
}


bool set_controller_gains(std::vector<double> cart_controller_gains,
                          std::vector<double> pendulum_controller_gains){   
    // set cart controller gains                          
    Kp_c = cart_controller_gains[0];
    Ki_c = cart_controller_gains[1];
    Kd_c = cart_controller_gains[2];

    // set pendulum controller gains
    Kp_p = pendulum_controller_gains[0];
    Ki_p = pendulum_controller_gains[1];
    Kd_p = pendulum_controller_gains[2];

    ROS_INFO("[tuned controller]: controller gains set");
    return true;
                          }


void reset_loss(){
    yaw_loss = 0;
    pos_loss = 0;
}

void reset_system_state(){
    std::lock_guard<std::mutex> lock(control_vars_mutex);
    current_pos = 0;
    yaw_rate = 0;
}

void zero_out_effort(){
    std_msgs::Float64 zero_effort_msg;
    zero_effort_msg.data = 0;
    cmd_pub.publish(zero_effort_msg);
}


void control_loop(double control_loop_rate){

    // set control variables
    double time_interval_val = 1 / control_loop_rate;
    ros::Rate control_rate(control_loop_rate);
    std_msgs::Float64 effort_msg;
    double effort = 0;

    // yaw variables 
    double last_yaw_error = 0.0,  integral_yaw_error = 0.0, effort_yaw = 0.0;
    double yaw_angle = 0.0, target_yaw = 0.0, yaw_error = 0.0;
    

    // position varibales 
    double target_pos = 0, pos_error = 0, last_pos_error = 0;
    double integral_pos_error = 0, effort_pos = 0; 
    

    // wait for the control start signal 
    while (!start_control){
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // check for the control start signal at 20 Hz
    }

    // control loop
    while ( ros::ok() && !break_control_loop ){
        
        ++control_call_count;
        
        // updating the state by invoking callbacks 
        // std::string queue_empty =  state_update_queue.empty() ? "yes" : "no";
        // ROS_INFO_THROTTLE(1,"[control]: queue empty: %s", queue_empty.c_str());
        // state_update_queue.callAvailable();
        ROS_INFO_THROTTLE(1,"[control]: %d", control_call_count);
       

        // compute control effort
        {
            std::lock_guard<std::mutex> lock(control_vars_mutex);
            yaw_angle += yaw_rate * time_interval_val; //update yaw angle 
        }
        yaw_error = target_yaw - yaw_angle;
        integral_yaw_error += (yaw_error + last_yaw_error) * time_interval_val / 2;
        effort_yaw  = -(Kp_p * yaw_error  + 
                    Ki_p * integral_yaw_error +
                    Kd_p * (yaw_error - last_yaw_error) / time_interval_val);   
        
        
        {
            std::lock_guard<std::mutex> lock(control_vars_mutex);
            pos_error = target_pos - current_pos;
            
        }
        integral_pos_error += (pos_error + last_pos_error) * time_interval_val / 2;
        effort_pos = -(Kp_c * pos_error  + 
                    Ki_c * integral_pos_error +
                    Kd_c * (pos_error - last_pos_error) / time_interval_val);   

        effort = effort_pos + effort_yaw;
        ROS_INFO_THROTTLE(1,"[control]: error_pos: %f, error_yaw: %f", pos_error, pos_error);
        ROS_INFO_THROTTLE(1,"[control]: effort_pos: %f, effort_yaw: %f", effort_pos, effort_yaw);
        effort_msg.data = effort;
        last_yaw_error = yaw_error;
        last_pos_error = pos_error;

        // accumulate errors
        yaw_loss += std::abs(yaw_error);
        pos_loss += std::abs(pos_error);
        
        cmd_pub.publish(effort_msg);
        control_rate.sleep(); // run at specified control rate

    }

    // send zero effort
    effort_msg.data = 0;
    cmd_pub.publish(effort_msg);

    // 
    
}


void start_controller(){
    start_control = true;
    break_control_loop = false;
    control_thread = std::thread(control_loop, 200.0);
}


void stop_controller(){
    break_control_loop = true;
    start_control = false;
    if (control_thread.joinable()){
        control_thread.join();
    }
    else{
        ROS_FATAL("[tuned controller]: control thread not joinable"); // FIXME: This is temporary else branch 
    }
}


void reset_gazebo(){
    if (reset_simulation_client.call(reset_simulation_srv)){
        ROS_INFO("[tuned controller]: simulation reset");
    }
    else {
        ROS_FATAL("[tuned controller]: failed to reset simulation");
    }
}

// define evaluate controller service callback 
bool evaluate_controller_callback(ipp::CandidateSolutionRequest& req,  
                                  ipp::CandidateSolutionResponse& res){

    // stop the control loop
    stop_controller();

    // set controller gains
    std::vector<double> pendulum_controller_gains = req.pendulum_controller_gains;
    std::vector<double> cart_controller_gains     = req.cart_controller_gains;
    set_controller_gains(cart_controller_gains, pendulum_controller_gains);

    // reset losses, effort, and system state 
    reset_loss();
    zero_out_effort();
    reset_system_state();

    ROS_INFO_THROTTLE(1,"random ass print");
    // reset the simulation  
    reset_gazebo();
    ros::Duration(0.005).sleep(); // wait for the simulation to reset

    // set simulation vars
    double simulation_end_time = 5.0;

    // signal control loop to satrt 
    start_controller();
                                  
    // run the controller for the specified duration
    while ( ros::Time::now().toSec() < simulation_end_time ){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        ROS_INFO_THROTTLE(1,"[tune controller/evaluate service]: time: %f, end_time: %f",
                          ros::Time::now().toSec(), simulation_end_time);
    }

    // stop the control loop
    stop_controller();

    // return loss
    res.loss_pend = yaw_loss;
    res.loss_cart = pos_loss;
    res.success   = true;

    return true;
}


int main(int argc, char** argv){

    // init node
    ros::init(argc,argv,"tuned_controller_node");

    // create a separate callback queue for updating system state 
    ros::NodeHandle state_update_nh;
    state_update_nh.setCallbackQueue(&state_update_queue);
    ros::Subscriber stats_update_subscriber = state_update_nh.subscribe(
        "/gazebo/link_states", 
        3, 
        &state_update_callback);

    // create a callback queue for dealing with service requests
    ros::CallbackQueue evaluate_controller_service_queue;
    ros::NodeHandle evaluate_controller_service_nh;
    evaluate_controller_service_nh.setCallbackQueue(&evaluate_controller_service_queue);
    ros::ServiceServer evaluate_controller_server =
    evaluate_controller_service_nh.advertiseService<
        ipp::CandidateSolutionRequest, 
        ipp::CandidateSolutionResponse>(
            "/evaluate_controller",
            &evaluate_controller_callback);

    // other publishers and service clients
    ros::NodeHandle nh;
    reset_simulation_client = nh.serviceClient<std_srvs::Empty>(
        "/gazebo/reset_simulation");
    cmd_pub = nh.advertise<std_msgs::Float64>(
        "/inverted_pendulum/joint_cart_controller/command", 
        1); 

    
    
    // ensure existence of gazebo reset simulation servie
    if (reset_simulation_client.waitForExistence(ros::Duration(5.0))){
        ROS_INFO("tunning controller]: reset simulation service found");
    }
    else{
        ROS_FATAL("[tuned controller]: reset simulation service not found during in 5 seconds. Exiting...");
    }

    // Add a dedicated async spinner for the state updates
    ros::AsyncSpinner state_update_spinner(1, &state_update_queue);
    state_update_spinner.start();

    // create a thread for the control loop
    std::thread control_loop_thread = std::thread(control_loop, 200.0);

    // spin the global callback queue for service requests
    ros::Rate evaluate_service_check_rate(10);
    while (ros::ok()){
        evaluate_controller_service_queue.callAvailable();
        evaluate_service_check_rate.sleep();
    }

    state_update_spinner.stop();
    control_loop_thread.join();

    return 0;
}