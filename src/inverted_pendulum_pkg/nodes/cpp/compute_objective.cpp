
#include "inverted_pendulum_pkg/UpdatePIDParamsRequest.h"
#include "inverted_pendulum_pkg/UpdatePIDParamsResponse.h"
#include "ros/duration.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "inverted_pendulum_pkg/UpdatePIDParams.h"
#include "inverted_pendulum_pkg/CandidateSolution.h"
#include "ros/subscriber.h"
#include "std_msgs/Float64.h"
#include "std_srvs/Empty.h"
#include "sensor_msgs/JointState.h"


#include "gazebo_msgs/ApplyBodyWrench.h"
#include "geometry_msgs/Pose.h"
#include"inverted_pendulum_pkg/ControlPoseData.h"
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <chrono>




class RunSim {

protected:

    ros::NodeHandle nh_;
    ros::ServiceServer server_;

    ros::Subscriber joint_state_sub_;
    //ros::Subscriber controller_cmd_sub_; //this subscribes to the node containing the desired setpoint

    ros::ServiceClient gazebo_reset_client_;
    ros::ServiceClient gazebo_pause_client_;
    ros::ServiceClient gazebo_unpause_client_;
    ros::ServiceClient controller_setgains_client_;
    ros::ServiceClient apply_body_wrench_client_;

    std_srvs::Empty empty_srv_;
    
    //declaring msgs and srvs that will be used for setting the PID gains
    inverted_pendulum_pkg::CandidateSolution sol_;
    inverted_pendulum_pkg::UpdatePIDParams update_pid_srv_;

    //decalaring msgs and variables that will offset the pendulum at the beginning of the simulation
    gazebo_msgs::ApplyBodyWrench app_body_wrench_msg_;
    geometry_msgs::Pose pend_offset_pose_msg_;
    const std::string pendulum_link_name_ = "inverted_pendulum::link_pendulum";
    
    

    ros::Duration simulation_time_; 
    ros::Duration wait_for_srv_time_out_;

    const float desired_cart_pos_ = 1.0; //TODO: I have to fix this. For now the position feedback is with respect to the world frame (hence, this is set to 1)
    const float desired_pend_pos_ = 0.0;
    float accum_track_error_pend_ , accum_track_error_cart_;
    //std::vector<float> track_error_hist_pend_ , track_error_hist_cart;
    
    //declaring variables to help accumlate the tracking error at a constat rate
    ros::Time last_time_;  //I dont think I need to create 
    ros::Time current_time_;
    const ros::Duration  accum_error_time_span_;
    
    



public:
    RunSim(int sim_time): accum_track_error_pend_(0.0), 
                          accum_track_error_cart_(0.0), 
                          last_time_(ros::Time::now()),
                          accum_error_time_span_(0.1)
                          {

        joint_state_sub_ = nh_.subscribe("/inverted_pendulum/control_pose_data",10,&RunSim::joint_state_CB,this); //TODO: I should add some logic to 
        //ensure that the topic exist 
        //controller_cmd_sub_ = nh_.subscribe("/inverted_pendulum/joint_cart_controller/command",10,);

        //Initialize the service server 
        const boost::function<bool(
                inverted_pendulum_pkg::CandidateSolutionRequest&,inverted_pendulum_pkg::CandidateSolutionResponse&)
                             > f_ = boost::bind(&RunSim::srvCB,this,_1,_2);

        server_ = nh_.advertiseService("inverted_pendulum/compute_objective",f_);
        gazebo_reset_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation",false);
        gazebo_pause_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/pause_physics",false);
        gazebo_unpause_client_ = nh_.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics",false);
        apply_body_wrench_client_ = nh_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench",false);
        controller_setgains_client_ = nh_.serviceClient<inverted_pendulum_pkg::UpdatePIDParams>("/inverted_pendulum/set_pid_gains"); //how can I modify this to work with other 
        //pkgs and ns

        simulation_time_ = ros::Duration(sim_time);
        wait_for_srv_time_out_ = ros::Duration(5); //5 seconds timeout when waiting for the above service clients to connect 

        set_app_body_wrench_msg();

        
        
    }   

    bool wait_for_servers(){
        

        ROS_INFO("Waiting for the clients to connect ...");

        bool reset_connect, pause_connect, unpause_connect, applywrench_connect, setgains_connect;

        
        reset_connect = gazebo_reset_client_.waitForExistence(wait_for_srv_time_out_);
        if (reset_connect)
            ROS_INFO("reset client connected ...");

        pause_connect = gazebo_pause_client_.waitForExistence(wait_for_srv_time_out_);
        if (pause_connect)
            ROS_INFO("pause client connected ...");

        unpause_connect = gazebo_unpause_client_.waitForExistence(wait_for_srv_time_out_);
        if (unpause_connect)
            ROS_INFO("unpause client connected ...");

        applywrench_connect = apply_body_wrench_client_.waitForExistence(wait_for_srv_time_out_);
        if (applywrench_connect)
            ROS_INFO("apply body wrench client connected ...");


        setgains_connect = controller_setgains_client_.waitForExistence(wait_for_srv_time_out_);
        if (setgains_connect)
            ROS_INFO("setgains client connected ...");

        return (reset_connect && pause_connect && unpause_connect && applywrench_connect && setgains_connect); 

    
    
    }

protected:

    void set_app_body_wrench_msg(){
    
        app_body_wrench_msg_.request.body_name = pendulum_link_name_;
        app_body_wrench_msg_.request.start_time = ros::Time::now() + ros::Duration(0.2);
        app_body_wrench_msg_.request.duration = ros::Duration(0.1); // I set this value after trial and error
        app_body_wrench_msg_.request.wrench.torque.y = 1.0;
    
    }

    void reset_accum_error(){

        accum_track_error_pend_ = 0;
        accum_track_error_cart_ = 0;
    
    }

    bool srvCB(inverted_pendulum_pkg::CandidateSolutionRequest& req,
               inverted_pendulum_pkg::CandidateSolutionResponse& resp)
               {
    
        bool success_flag = true; //will be used to indicate the success of the srv
        std::vector<double> pend_cont_gains, cart_cont_gains;
        
        pend_cont_gains = req.pendulum_controller_gains;
        cart_cont_gains = req.cart_controller_gains;

        ROS_INFO("The recieved pid gains are: (pendulum): %f, %f, %f | (cart):, %f, %f, %f"
                ,pend_cont_gains[0],pend_cont_gains[1],pend_cont_gains[2],
                cart_cont_gains[0],cart_cont_gains[1],cart_cont_gains[2]);
        set_update_pid_msg(pend_cont_gains ,cart_cont_gains);
        
        //send the reconfig service to the server
        if (controller_setgains_client_.call(update_pid_srv_)){  //
        
            ROS_INFO("The gains were set to the updated values!");

        }
        else {
        return false; //failed to set the pid gains 
        }
            

        //I should run the simulation here 
        ROS_INFO("Resetting simulation ...");
        if(gazebo_reset_client_.call(empty_srv_))
            ROS_INFO("Simulation has been reset ...");
        else{
            ROS_INFO("Was not able to reset simualtion :(");
            return false;
        }

        reset_accum_error();


         ROS_INFO("Unpausing simulation");
         if(gazebo_unpause_client_.call(empty_srv_))
            ROS_INFO("Simulation has been unpaused ...");
        else{
            ROS_INFO("Was not able to unpause simualtion :(");
            return false;
        }

        ROS_INFO("Offseting the pendulum from equilibrium");
        if (apply_body_wrench_client_.call(app_body_wrench_msg_))
            ROS_INFO("Setting the offset was successful!");
        else {
            ROS_INFO("Setting offset was not successful :(");
            return false;
        }

        // ROS_INFO("Sleeping for 2 seconds");
        // // ros::WallDuration(2).sleep();
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        // ROS_INFO("A message to check the sleep");
        


       

        if (success_flag){  //FIXME: I believe this is poitless as the flag is always true //if the simulation has been reset and the PID has been updated, run the simulation 
           
            ROS_INFO("Simulation will now run for %f seconds.", simulation_time_.toSec());
            ros::Time sim_start_time = ros::Time::now();
            ros::Time current_time = ros::Time::now();
            
            //this controls the tracking error accumulation rate
            last_time_ = sim_start_time;


            while((current_time - sim_start_time) < simulation_time_){
            

                current_time = ros::Time::now();
                ros::spinOnce();
            
            
            
            }

        ros::Duration temp = current_time - sim_start_time;    
        ROS_WARN("the simulation was running for %f seconds",temp.toSec());
        resp.loss_pend =  accum_track_error_pend_; //accum_track_error_cart TODO: I am commenting this part for now
        resp.loss_cart = accum_track_error_cart_;
        resp.success = true;
        
        }

        

        // ROS_INFO("Pausing the simulation for the next run..");
        // if (gazebo_pause_client_.call(empty_srv_))
        //     ROS_INFO("Pausing the simulation was succesful!");
        
        
        return success_flag;

    
    
    }
    

    void set_update_pid_msg(const std::vector<double>& pend_cont_gains,const std::vector<double>& cart_cont_gains){

        ROS_INFO("Setting the pid parameters ...");
    
        update_pid_srv_.request.pendulum_controller_gains = pend_cont_gains;
        update_pid_srv_.request.cart_controller_gains = cart_cont_gains;
        

    
    }

    void joint_state_CB(const inverted_pendulum_pkg::ControlPoseDataConstPtr& msg){
        
        // ROS_INFO("This is the callback ...");
        double pend_pos = msg->pendulum_yaw;
        double cart_pos = msg->cart_posx;
        current_time_ = ros::Time::now();
        ros::Duration t_span = current_time_ - last_time_;
        // ROS_INFO("del time is %f",t_span.toSec());

        if ((current_time_ - last_time_) > accum_error_time_span_) {
            ROS_INFO("Accumalating error...");
            accum_track_error_pend_ += 4 * std::abs(static_cast<double>(desired_pend_pos_) - pend_pos)  * 1.0; // FIXEM: I temporarily scaled the pendulum error by four to add more cost
                                        // static_cast<double>(current_time_.toNSec()); // I made the cost grow with time

            accum_track_error_cart_ += std::abs(static_cast<double>(desired_cart_pos_) - cart_pos) * 1.0;
                                        // static_cast<double>(current_time_.toNSec());
            
            // print the error to the console for debugging 
            ROS_INFO("pend error: %f", accum_track_error_pend_);
            ROS_INFO("cart error: %f", accum_track_error_cart_);
            
            last_time_ = current_time_;
        }

        

    
    }


};


int main(int argc, char** argv) {


    ros::init(argc,argv,"compute_objective_node");
    
    int sim_time = std::stoi(argv[1]);
    ROS_INFO("The simulation time is %d", sim_time);
    

    
    
    RunSim run_sim(sim_time);

    bool all_clients_connected = false;
    

    while(!all_clients_connected){

        all_clients_connected = run_sim.wait_for_servers();
        ROS_INFO("flag is %d",all_clients_connected);

    }

    ROS_INFO("All clients connected to their servers!");
   

    while (ros::ok()){
    
        ros::spinOnce();
    
    }


}