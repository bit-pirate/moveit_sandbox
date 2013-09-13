#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup::Options options("arm", "robot_description");
  ROS_INFO_STREAM("Configuring move group interface ...");
  // this connecs to a running instance of the move_group node
  moveit::planning_interface::MoveGroup group(options);
  ROS_INFO_STREAM("Move group interface configured.");

  // specify that our target will be a random one
//  ROS_INFO_STREAM("Setting random target ...");
//  group.setRandomTarget();
//  // plan the motion and then move the group to the sampled target
//  ROS_INFO_STREAM("Moving arm ...");
//  if (group.move())
//  {
//    ROS_INFO_STREAM("Moving done.");
//  }
//  else
//  {
//    ROS_WARN_STREAM("Moving failed.");
//  }

  // specifying a goal pose
  ROS_INFO_STREAM("Setting pose goal for end effector '" << group.getEndEffectorLink() << "'.");
  geometry_msgs::Pose goal_pose;
  // the pose given by the values below is valid
  goal_pose.position.x = 0.5;
  goal_pose.position.y = 0.0;
  goal_pose.position.z = 0.5;
  goal_pose.orientation.x = 0.70711;
  goal_pose.orientation.y = 0.0;
  goal_pose.orientation.z = 0.0;
  goal_pose.orientation.w = 0.70711;
  ROS_INFO_STREAM("Pose goal:");
  ROS_INFO_STREAM(goal_pose);

  group.setPoseTarget(goal_pose);
  group.setGoalPositionTolerance(1e-5);
  group.setGoalOrientationTolerance(1e-5);
  group.setPlanningTime(1.0);
  if (group.move())
  {
    ROS_INFO_STREAM("Moving done.");
  }
  else
  {
    ROS_WARN_STREAM("Moving failed.");
  }

  spinner.stop();
//  ros::waitForShutdown();
}
