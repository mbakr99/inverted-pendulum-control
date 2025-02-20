#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "std_msgs/Float64.h"
#include <vector>
#include "std_srvs/Empty.h"
#include "gazebo_msgs/LinkStates.h"
#include <thread>
#include <mutex>



// Global variables 
double Kp_c, Ki_c, Kd_c, Kp_p, Ki_p, Kd_p;
double yaw_rate  = 0, current_pos = 0;

// debug variables 
int num_available_callbacks = 0;
int control_call_count      = 0; 
int update_call_count       = 0;


// thread safety 
std::mutex control_vars_mutex;


void state_update_callback(const gazebo_msgs::LinkStatesConstPtr& state_msg){

    ROS_INFO_THROTTLE(1,"[update]: %d", ++update_call_count);

    // get cart-pole pose data
    std::vector<std::string> link_names_ = state_msg->name;

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



int main(int argc, char** argv){

    // init node
    ros::init(argc,argv,"tuned_controller_node");

    // set publisher and subscribers
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<std_msgs::Float64>("/inverted_pendulum/joint_cart_controller/command", 1); 
    ros::Subscriber state_update_sub = nh.subscribe("/gazebo/link_states", 3, &state_update_callback);
    
    // set control variables
    double effort = 0, time_interval_val = 0.005;
    ros::Rate control_rate(1 / time_interval_val);
    Kp_c = 00, Ki_c = 06.18, Kd_c = 5.50;
    Kp_p = 18, Ki_p = 19.34, Kd_p = 7.75;
    std_msgs::Float64 effort_msg;
    
    // yaw variables 
    double last_yaw_error = 0.0,  integral_yaw_error = 0.0, effort_yaw = 0.0;
    double yaw_angle = 0.0, target_yaw = 0.0, yaw_error = 0.0;
    

    // position varibales 
    double target_pos = 0, pos_error = 0, last_pos_error = 0;
    double integral_pos_error = 0, effort_pos = 0; 

    // control loop
    while (ros::ok()){
        
        ++control_call_count;
        
        // process all available callbacks
        while (!ros::getGlobalCallbackQueue()->empty()){
            ros::getGlobalCallbackQueue()->callOne();
            ++ num_available_callbacks;
        }

        ROS_INFO_THROTTLE(1,"[control]: %d", control_call_count);
        ROS_INFO_THROTTLE(1,"[control]: # available callbacks: %d", num_available_callbacks);
        num_available_callbacks = 0;

        // compute control effort
        yaw_angle += yaw_rate * time_interval_val; //update yaw angle 
        yaw_error = target_yaw - yaw_angle;
        integral_yaw_error += (yaw_error + last_yaw_error) * time_interval_val / 2;
        effort_yaw  = -(Kp_p * yaw_error  + 
                    Ki_p * integral_yaw_error +
                    Kd_p * (yaw_error - last_yaw_error) / time_interval_val);   

        pos_error = target_pos - current_pos;
        integral_pos_error += (pos_error + last_pos_error) * time_interval_val / 2;
        effort_pos = -(Kp_c * pos_error  + 
                    Ki_c * integral_pos_error +
                    Kd_c * (pos_error - last_pos_error) / time_interval_val);   

        effort = effort_pos + effort_yaw;
        effort_msg.data = effort;
        last_yaw_error = yaw_error;
        last_pos_error = pos_error;
        
        cmd_pub.publish(effort_msg);
        control_rate.sleep(); // run at specified control rate
    }

    // send zero effort
    effort_msg.data = 0;
    cmd_pub.publish(effort_msg);

    return 0;
}