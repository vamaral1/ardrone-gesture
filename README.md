## Gesture-based control of ArDrone

1. Install OpenNI for Ros sudo apt-get install ros-groovy-openni-*
2. Install SensorKinect https://github.com/avin2/SensorKinect/tree/unstable
3. Install OpenNI https://github.com/OpenNI/OpenNI/tree/unstable
4. Install Nite https://simple-openni.googlecode.com/files/OpenNI_NITE_Installer-Linux64-0.27.zip
5. Run roscore, rosrun openni_tracker openni_tracker, roslaunch openni_launch openni.launch
6. Tracker publishes to tf, rostopic echo /tf, rosrun rviz rviz to visualize it
7. In rviz, add tf to the display frames, and change the "fixed frame" to openni_depth_frame
8. rosrun ardrone_autonomy ardrone_driver -ip 192.168.1.200
9. publish empty message: rostopic pub /ardrone/takeoff std_msgs/Empty

right hand (actually left hand topic) - up turns left, down turns right

left hand - velocity is increased as its moved horizontally away from the neck





