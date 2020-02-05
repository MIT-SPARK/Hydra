#include <Eigen/Core>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros/ros.h>

#include <object_db/ObjectRegistrationAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teaser_sample_client");

  ROS_INFO("Starting client.");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<object_db::ObjectRegistrationAction> ac(
      "object_db", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); // will wait for infinite time

  ROS_INFO("TEASER++ server started.");

  return 0;
}
