#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_demo", ros::init_options::AnonymousName);
  // start a ROS spinning thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup::Options options("arm", "/robot_description");
  ROS_INFO_STREAM("Configuring move group interface ...");
  // this connecs to a running instance of the move_group node
  moveit::planning_interface::MoveGroup group(options);
  ROS_INFO_STREAM("Move group interface configured.");
  // specify that our target will be a random one
  ROS_INFO_STREAM("Setting random target ...");
  group.setRandomTarget();
  // plan the motion and then move the group to the sampled target
  ROS_INFO_STREAM("Moving arm ...");
  if (group.move())
  {
    ROS_INFO_STREAM("Moving done.");
  }
  else
  {
    ROS_WARN_STREAM("Moving failed.");
  }

  ros::waitForShutdown();
}
