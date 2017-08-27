#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>

int main(int argc, char** argv){
    ROS_INFO("Controlling ArDrone with Asus Xtion Pro");
    ros::init(argc, argv, "ardrone_tf_listener");

    ros::NodeHandle node;
    ros::Rate rate(20);
    std_msgs::Empty emp_msg;
    geometry_msgs::Twist twist_msg_hover;
    twist_msg_hover.linear.x=0.0; 
    twist_msg_hover.linear.y=0.0;
    twist_msg_hover.linear.z=0.0;
    twist_msg_hover.angular.x=0.0; 
    twist_msg_hover.angular.y=0.0;                          
    twist_msg_hover.angular.z=0.0;

    float takeoff_time=10.0;
    float land_time=3.0;
    float kill_time =2.0;  
    float fly_time =60.0;   

    tf::TransformListener listener;
    ros::Publisher pub_empty_land;
    ros::Publisher pub_twist;
    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_reset;
    double start_time;

    //Initializes the publishers, message queue length is just 1 */
    pub_twist = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1); 
    pub_empty_land = node.advertise<std_msgs::Empty>("/ardrone/land", 1); 
    pub_empty_reset = node.advertise<std_msgs::Empty>("/ardrone/reset", 1); 
    
    start_time =(double)ros::Time::now().toSec();   
    ROS_INFO("Starting ARdrone_test loop");

  while (node.ok()){
	while ((double)ros::Time::now().toSec()< start_time+takeoff_time){ //takeoff
		
			pub_empty_takeoff.publish(emp_msg); //launches the drone
				pub_twist.publish(twist_msg_hover); //drone is flat
			ROS_INFO("Taking off");
			//calls all the callbacks waiting to be called at this time
            		ros::spinOnce();
			rate.sleep();
	}//while takeoff

	while ( (double)ros::Time::now().toSec()> start_time+takeoff_time && 						(double)ros::Time::now().toSec()< start_time+takeoff_time+fly_time){	
		//ROS_INFO("Controlling via Xtion");
		tf::StampedTransform right;
    		tf::StampedTransform left;
    		try{
     			listener.lookupTransform("/neck_1", "/left_hand_1",ros::Time(0), right);
      			listener.lookupTransform("/neck_1", "/right_hand_1",ros::Time(0), left);
    			}
    		catch (tf::TransformException &ex) {
     			ROS_ERROR("%s",ex.what());
      			ros::Duration(1.0).sleep();
      			continue;
    		}
    
		geometry_msgs::Twist vel_msg;
        /*double rotz = -atan2(right.getOrigin().x()-0.2,right.getOrigin().y())*0.5;
        if(abs(rotz) > 0.1) {
		    vel_msg.angular.z = rotz;
        } else { vel_msg.angular.z = 0.0; }*/
        //rotation about z axis specified in radians per second
        vel_msg.angular.z = 0.0;
        //forward/backwards
        vel_msg.linear.x =  left.getOrigin().y()*0.4;
        //left/right
   		vel_msg.linear.y = - right.getOrigin().x()*0.4;
        //up/down
    	vel_msg.linear.z = 0.0;
    	vel_msg.angular.x = 0.0; 
    	vel_msg.angular.y = 0.0;                          
    	pub_twist.publish(vel_msg);
		
		ros::spinOnce();
		rate.sleep();
	}

		while  ((double)ros::Time::now().toSec()> start_time+takeoff_time){
		
			pub_twist.publish(twist_msg_hover); //drone is flat
			pub_empty_land.publish(emp_msg); //lands the drone
			ROS_INFO("Landing");
			
					
			if ((double)ros::Time::now().toSec()> takeoff_time+start_time+land_time+fly_time+kill_time){
		
				ROS_INFO("Closing Node");
				//pub_empty_reset.publish(emp_msg); //kills the drone		
				exit(0); 	
			}//kill node
			ros::spinOnce();
			rate.sleep();	
	}
    
    rate.sleep();
  }
}
