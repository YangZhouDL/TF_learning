#ifndef TRY_TF_LISTENER_CLASS_H
#define TRY_TF_LISTENER_CLASS_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;

class tfListener
{
private:
    ros::NodeHandle n;
    // tf::TransformListener *tf_listener;
public:
    tfListener(ros::NodeHandle& nodehandle);
    tf::TransformListener *tf_listener;

    bool multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf,tf::StampedTransform &C_stf);

    tf::Transform get_tf_from_stamped_tf(tf::StampedTransform sTf);
    void printTf(tf::Transform tf); 

    void printStampedTf(tf::StampedTransform sTf); 

    geometry_msgs::PoseStamped get_pose_from_transform(tf::StampedTransform tf);       
    void printStampedPose(geometry_msgs::PoseStamped stPose); 
};

#endif