#include "gazebo_msgs/LinkStates.h"
#include "inverted_pendulum_pkg/ControlPoseData.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include "ros/subscriber.h"
#include <vector>

class UpdatePendulumPose {

private:
  ros::NodeHandle nh_;
  ros::Subscriber link_states_sub_;
  ros::Publisher pend_pose_pub_;

  ros::Time last_time_;
  ros::Time current_time_;
  ros::Duration time_span_;

  double current_yaw_;

public:
  UpdatePendulumPose(double init_yaw) current_yaw(init_yaw),
      last_time_(ros::Time::now()) {

    link_states_sub_ = nh_.subscribe("/gazebo/link_states", 10,
                                     &UpdatePendulumPose::subCB, this);
  }

private:
  subCB(const gazebo_msgs::LinkStatesConstPtr &msg) {

    std::msg->

    // current_time_ = ros::Time::now();
    // time_span_ = current_time_ - last_time_;

    // current_yaw_ += time_span_
  }

}




int main(int argc, char** argv){

  ros::init(argc, argc, "update_cart_pose_data");
}

