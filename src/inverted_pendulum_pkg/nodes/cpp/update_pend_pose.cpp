#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "inverted_pendulum_pkg/ControlPoseData.h"
#include "ros/service_client.h"
#include "ros/subscribe_options.h"
#include "ros/subscriber.h"
#include "gazebo_msgs/LinkStates.h"
#include "gazebo_msgs/GetLinkState.h"
#include <vector>


class UpdatePendulumPose{


private:

    // Declaring subscribers, publoshers, and clients
    ros::NodeHandle nh_;
    ros::Subscriber link_states_sub_;
    ros::Publisher my_custom_publisher_;
    ros::ServiceClient get_link_state_client_;

    // Auxilary data needed when listening to link_states topic
    std::vector<std::string> link_names_;
    const std::string cart_link_name_ = "cart_pole::cart_link";
    const std::string pend_link_name_ = "cart_pole::pole_link";
    
    // Variables to store yaw and cart x position data
    double current_yaw_rate;
    double current_yaw_;
    double current_cart_posx_;

    // Yaw integration variables 
    ros::Duration yaw_update_time;
    ros::Time last_time;


    //Auxilary data to publish the custom msg that contains only the pendulum yaw and cart pos (This can be modified later to include other things if needed)
    inverted_pendulum_pkg::ControlPoseData my_custom_msg_;  
     





public:

    UpdatePendulumPose() : last_time(ros::Time::now()), yaw_update_time(ros::Duration(0.05)),
                           current_yaw_(0), current_yaw_rate(0), current_cart_posx_(0)
    {
        link_states_sub_ = nh_.subscribe("gazebo/link_states",10,&UpdatePendulumPose::subCB,this);
        my_custom_publisher_ = nh_.advertise<inverted_pendulum_pkg::ControlPoseData>("inverted_pendulum/control_pose_data",10,false);
        ROS_INFO("UpdatePendulumPose object has been created!"); 
    }

private:


    void subCB(const gazebo_msgs::LinkStatesConstPtr& msg){
        // Get cart position data 
        link_names_ = msg->name;

        int index = -1;
        for (int i = 0; i < link_names_.size(); ++i){
            if (link_names_[i] == cart_link_name_){
                index = i;
                break;
            }
        }

        current_cart_posx_ = msg->pose[index].position.x; // store x cart position

        //Get the pole yaw by calling the get link
        index = -1;
        for (int i = 0; i < link_names_.size(); ++i){
            if (link_names_[i] == pend_link_name_){
                index = i;
                break;
            }
        }

        // integrate yaw rate
        current_yaw_rate = msg->twist[index].angular.y; 
        ros::Time current_time = ros::Time::now();
        ros::Duration dt = current_time - last_time;

        if (last_time.isZero() || dt.toSec() < 0) {
            ROS_WARN_THROTTLE(1.0, "Invalid dt detected (%f), resetting last command time", dt.toSec());
            last_time = current_time;
            return;
        }

        if (dt >= yaw_update_time){
            current_yaw_ += current_yaw_rate * dt.toSec();
            last_time = current_time;
        }


        
        // publish pose data 
        my_custom_msg_.pendulum_yaw = current_yaw_;
        my_custom_msg_.cart_posx = current_cart_posx_;
        my_custom_publisher_.publish(my_custom_msg_);
   
    }


};




int main(int argc, char** argv){

ros::init(argc,argv,"update_pend_pose_data");

UpdatePendulumPose update_pend_pose;
ROS_INFO("Running the update_pend_pose_data node!");

while (ros::ok()){

    ros::spinOnce();

}









}