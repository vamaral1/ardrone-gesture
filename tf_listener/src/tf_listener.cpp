#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_listener");

    ros::NodeHandle node;

    tf::TransformListener listener;

    ros::service::waitForService("spawn");
    ros::ServiceClient add_turtle = 
                node.serviceClient<turtlesim::Spawn>("spawn");          turtlesim::Spawn srv;
    add_turtle.call(srv);
    ros::Publisher turtle_vel = 
                      node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);


  ros::Rate rate(10.0);
  while (node.ok()){
  //for(;;) {
    tf::StampedTransform rotate;
    tf::StampedTransform vertical;
    try{
      listener.lookupTransform("/torso_1", "/left_hand_1",ros::Time(0), rotate);
      listener.lookupTransform("/torso_1", "/right_hand_1",ros::Time(0), vertical);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

   // std::cout << "y:" << transform.getOrigin().y() << "\nx: " << transform.getOrigin().x() << "\nz:"<< transform.getOrigin().z();
    
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = -atan2(rotate.getOrigin().x()-0.2,rotate.getOrigin().y()) * 0.5;
    vel_msg.linear.x = vertical.getOrigin().y()*0.7;
    vel_msg.linear.y = vertical.getOrigin().z()*0.7;
    turtle_vel.publish(vel_msg);
    
    rate.sleep();
  }
  return 0;
};
