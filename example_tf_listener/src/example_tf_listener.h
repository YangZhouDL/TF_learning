// example_tf_listener.h header file //
// wsn; March, 2016
// include this file in "example_tf_listener.cpp"

#ifndef EXAMPLE_TF_LISTENER_H_
#define EXAMPLE_TF_LISTENER_H_

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>
//#include <tf/LinearMath/QuadWord.h>


// define a class, including a constructor, member variables and member functions
class DemoTfListener
{
private:
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor  
public:
    DemoTfListener(ros::NodeHandle& nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    //illustrates a tf_listener in a class; this is somewhat more complex than creating a tf_listener in main()
    tf::TransformListener* tfListener_;  
    
    bool multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf,tf::StampedTransform &C_stf); 

    void printStampedTf(tf::StampedTransform sTf); 
    
    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
    void printTf(tf::Transform tf);   
     
    geometry_msgs::PoseStamped get_pose_from_transform(tf::StampedTransform tf);       
    void printStampedPose(geometry_msgs::PoseStamped stPose);          
}; 
#endif  