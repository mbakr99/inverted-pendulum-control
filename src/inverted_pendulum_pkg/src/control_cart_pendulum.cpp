#include "control_toolbox/pid.h"
#include "inverted_pendulum_pkg/UpdatePIDParams.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/ros.h"
#include "inverted_pendulum_pkg/UpdatePIDParams.h"
#include "ros/subscriber.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "inverted_pendulum_pkg/ControlPoseData.h"
#include <cstdint>
#include <vector>
#include <array>



class InvPenController {

protected:

    control_toolbox::Pid cart_pos_controller_;
    control_toolbox::Pid pend_pos_controller_;

    ros::NodeHandle nh_;
    //ros::Subscriber setpoint_sub_; I will remove this for now
    ros::Subscriber joint_state_sub_;
    ros::Publisher cmd_pub_; //this will publish to the command topic of the JointEfforController from gazebo_ros_control
    ros::ServiceServer set_gains_server_;


    double pc_,ic_,dc_,pp_,ip_,dp_;
    const double set_point_pend_ = 0.0;
    const double set_point_cart_ = 1.0;
    double controller_cmd_;

    std::vector<double> cart_cont_gains_;
    std::vector<double> pend_cont_gains_;

    std_msgs::Float64 controller_cmd_msg_;

    ros::Time last_command_print_time_;
    ros::Time last_command_update_time_;
    ros::Duration command_print_rate_; 
    ros::Duration command_update_rate_;



public:

    InvPenController(double pc,double ic,double dc,
                     double pp,double ip,double dp,
                     double update_rate) :
                     pend_pos_controller_(),
                     cart_pos_controller_(),
                     last_command_print_time_(ros::Time::now()),
                     last_command_update_time_(ros::Time::now()),
                     command_print_rate_(ros::Duration(0.2)),
                     command_update_rate_(update_rate)
                     {
    
    
        //setpoint_sub_ = nh_.subscribe("inverted_pendulum/setpoint",1,&InvPenController::setpointCB,this);
        joint_state_sub_ = nh_.subscribe("/inverted_pendulum/control_pose_data",1,&InvPenController::joint_state_CB,this);
        cmd_pub_ = nh_.advertise<std_msgs::Float64>("/inverted_pendulum/joint_cart_controller/command",1,false);
        const boost::function<bool(inverted_pendulum_pkg::UpdatePIDParams::Request&, inverted_pendulum_pkg::UpdatePIDParams::Response&)> _f = boost::bind(&InvPenController::serviceCB,this,_1,_2);
        set_gains_server_ = nh_.advertiseService("/inverted_pendulum/set_pid_gains",_f);
    }



protected:


    double compute_command(double error_cart,double error_pend, ros::Duration dt) {
    
      return (cart_pos_controller_.computeCommand(error_cart, dt) -
            pend_pos_controller_.computeCommand(error_pend, dt)); 
    
    }


//void Pid::setGains(double p, double i, double d, double i_max, double i_min, bool antiwindup)
// {
//   Gains gains(p, i, d, i_max, i_min, antiwindup);

//   setGains(gains);
// }
    void set_gains(std::vector<double>& pend_cont_gains, std::vector<double>& cart_cont_gains ){
    
        //My implementation does not take into account i_max and i_min from the control_toolbox library 
        pp_ = pend_cont_gains[0];
        ip_ = pend_cont_gains[1];
        dp_ = pend_cont_gains[2];

        pc_ = cart_cont_gains[0];
        ic_ = cart_cont_gains[1];
        dc_ = cart_cont_gains[2];

        double i_min, i_max;
        i_min = 0; //TODO: I have to revisit this to make sure this is not causing a bug
        i_max = 1000;
        bool antiwindup = true;

        //ROS_INFO("Calling the setGains method from the control_toolbox");

        pend_pos_controller_.setGains(pp_,dp_,ip_,i_max,i_min,antiwindup);
        cart_pos_controller_.setGains(pc_,dc_,ic_,i_max,i_min,antiwindup);

        ROS_INFO("Gains has been chnaged to the requested values.");
    
    
    }



protected: //all of the class callbacks

    // void setpointCB(const std_msgs::Float64MultiArrayConstPtr& set_point_msg){
    //     //set the desired setpoints 
    //     set_point_pend_ = set_point_msg->data[0];
    //     set_point_cart_ = set_point_msg->data[1];
    //     ROS_INFO("The desired pendulum and cart positions are set to %f, and %f",set_point_pend_,set_point_cart_);

        

    // }

    void joint_state_CB(const inverted_pendulum_pkg::ControlPoseDataConstPtr& msg){
        //compute the pendulum and cart errors
        double error_pend = set_point_pend_ - msg->pendulum_yaw;
        double error_cart = set_point_cart_ - msg->cart_posx;

        ros::Time current_time = ros::Time::now();
        
       

        //I might want to add a line here to control the update rate (use if condition with )
        ros::Duration dt = current_time - last_command_print_time_;
        

        if (dt >= command_update_rate_){
            
            // compute cmd 
            controller_cmd_ = compute_command(error_cart, error_pend, dt);
            controller_cmd_msg_.data = controller_cmd_;

            // publish cmd
            cmd_pub_.publish(controller_cmd_msg_);
            last_command_update_time_  = ros::Time::now();
        }

        
        if ((current_time - last_command_print_time_) >= command_print_rate_){ 
            ROS_INFO("Sending the following command: %f",controller_cmd_);
            last_command_print_time_ = ros::Time::now();
        }

        

    }



    bool serviceCB(inverted_pendulum_pkg::UpdatePIDParams::Request& req, inverted_pendulum_pkg::UpdatePIDParams::Response& resp){

        
        pend_pos_controller_.reset();
        cart_pos_controller_.reset();

        ROS_INFO("Controllers have been reset!");

        pend_cont_gains_ = req.pendulum_controller_gains;
        cart_cont_gains_ = req.cart_controller_gains;
        ROS_INFO("The recieved pend gains are: %f, %f, %f",pend_cont_gains_[0],pend_cont_gains_[1],pend_cont_gains_[2]);
        ROS_INFO("The recieved cart gains are: %f, %f, %f",cart_cont_gains_[0],cart_cont_gains_[1],cart_cont_gains_[2]);
        
        //ROS_INFO("Calling my set_gains method");

        set_gains(pend_cont_gains_, cart_cont_gains_);

        resp.success = true;

        return true;

    
    
    }

  








};


int main(int argc, char** argv){

    ros::init(argc,argv,"controller_node");

    InvPenController inv_pend_controller(1,1,1,1,1,1,0.1);

    while (ros::ok()){
        ros::spinOnce();
    }

}