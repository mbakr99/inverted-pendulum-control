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

    //Declaring subscribers, publoshers, and clients
    ros::NodeHandle nh_;
    ros::Subscriber link_states_sub_;
    ros::Publisher my_custom_publisher_;
    ros::ServiceClient get_link_state_client_;



    //Auxilary data needed when listening to link_states topic
    std::vector<std::string> link_names_;
    const std::string cart_link_name_ = "inverted_pendulum::link_cart";
    

    //Variables to store yaw and cart x position data
    double current_yaw_;
    double current_cart_posx_;

    //Auxilary data needed to call the get_link_state service 
    const std::string pend_link_name_ = "link_pendulum";
    const std::string pend_ref_link_name_ = "link_cart";
    gazebo_msgs::GetLinkState srv_msg_;


    //Auxilary data to publish the custom msg that contains only the pendulum yaw and cart pos (This can be modified later to include other things if needed)
    inverted_pendulum_pkg::ControlPoseData my_custom_msg_;  
     





public:

    UpdatePendulumPose() {
     
        link_states_sub_ = nh_.subscribe("gazebo/link_states",10,&UpdatePendulumPose::subCB,this);
        get_link_state_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state",false);
        my_custom_publisher_ = nh_.advertise<inverted_pendulum_pkg::ControlPoseData>("inverted_pendulum/control_pose_data",10,false);
        init_srv_msg();
        ROS_INFO("UpdatePendulumPose object has been created!");
    
    
    
    }

 

private:


    void init_srv_msg(){
    
        srv_msg_.request.link_name = pend_link_name_;
        srv_msg_.request.reference_frame = pend_ref_link_name_;

    
    }


    void subCB(const gazebo_msgs::LinkStatesConstPtr& msg){
    
        //Get cart position data 
        link_names_ = msg->name;

        int index = -1;

        for (int i = 0; i < link_names_.size(); i++){
        
            if (link_names_[i]==cart_link_name_)
                index = i;
        }


        current_cart_posx_ = msg->pose[index].position.x;

        //Get the pole yaw by calling the get link
        if (get_link_state_client_.call(srv_msg_)){
        
            //ROS_INFO("Called the get_link_state service succesfully!");
            current_yaw_ = srv_msg_.response.link_state.pose.orientation.y;
        }
        else {
            ROS_INFO("Failed to call the get_link_state service and obtain the yaw ");
            current_yaw_ = -1;
        }

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