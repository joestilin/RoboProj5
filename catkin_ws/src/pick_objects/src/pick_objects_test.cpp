#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goto_goal(double x, double y, std::string goal_name) {

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Robot successfully reached %s\n", goal_name.c_str());
  else
    ROS_INFO("The base failed to reach goal");
}

// test by sending a pickup goal, waiting 5 seconds, and 
// sending a dropoff goal

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

	ros::NodeHandle n;
	
	// pickup and dropoff locations
	double pickup_x = 6.0;
	double pickup_y = 6.0;
	double pickup_z = 0.0;
  double dropoff_x = 2.0;
  double dropoff_y = -4.0;
	double dropoff_z = 0.0;

	// go to the pickup location
  goto_goal(pickup_x, pickup_y, "pickup zone");

	// wait for 5 seconds
  ros::Duration(5).sleep();

  // go to the dropoff location
  goto_goal(dropoff_x, dropoff_y, "dropoff zone");

  ros::Duration(5).sleep();  

	ros::spin();

  return 0;
}
