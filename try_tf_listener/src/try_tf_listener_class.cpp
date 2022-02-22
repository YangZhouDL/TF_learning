#include "try_tf_listener_class.h"
using namespace std;

tfListener::tfListener(ros::NodeHandle& nodehandle)
{
    ROS_INFO("in class constructor of tfListener_class");
    n=nodehandle;
    tf_listener= new tf::TransformListener;  
    
    bool tferr=true;
    ROS_INFO("waiting for tf between link2 and base_link...");
    tf::StampedTransform tfLink2_to_BaseLink; 

    while (tferr) 
    {
        tferr=false;
        try 
        {
            tf_listener->lookupTransform("base_link", "link2", ros::Time(0), tfLink2_to_BaseLink);
        } 
        catch(tf::TransformException &exception) 
        {
            ROS_WARN("%s; retrying...", exception.what());
            tferr=true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();                
        }   
    }

    ROS_INFO("tf is good");    
}

bool tfListener::multiply_stamped_tfs(tf::StampedTransform A_stf, tf::StampedTransform B_stf,tf::StampedTransform &C_stf)
{
    tf::Transform A,B,C; 
    std::string str1 (A_stf.child_frame_id_); 
    std::string str2 (B_stf.frame_id_);

    if (str1.compare(str2) != 0) 
    { 
        std::cout<<"can't multiply transforms; mismatched frames"<<endl;
        std::cout << str1 << " is not " << str2 << '\n'; 
        return false;
    }
    else
    {
        A = get_tf_from_stamped_tf(A_stf); 
        B = get_tf_from_stamped_tf(B_stf);
        C = A*B; 
        C_stf.frame_id_ = A_stf.frame_id_; 
        C_stf.child_frame_id_ = B_stf.child_frame_id_;
        C_stf.setOrigin(C.getOrigin()); 
        C_stf.setBasis(C.getBasis());
        C_stf.stamp_ = ros::Time::now(); 
        return true;
    }   
}

tf::Transform tfListener::get_tf_from_stamped_tf(tf::StampedTransform sTf)
{
    tf::Transform tf(sTf.getBasis(),sTf.getOrigin()); 
    // tf.setBasis(sTf.getBasis());
    // tf.setOrigin(sTf.getOrigin());
    return tf;
}

void tfListener::printTf(tf::Transform tf)
{
    Vector3d p(tf.getOrigin());
    cout<<"vector from reference frame to to child frame: "<<p.transpose()<<endl;
    
    tf::Matrix3x3 tfR=tf.getBasis();
    tf::Vector3 tfVec;
    cout<<"orientation of child frame w/rt reference frame: "<<endl;
    tfVec = tfR.getRow(0);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;
    tfVec = tfR.getRow(1);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;    
    tfVec = tfR.getRow(2);
    cout<<tfVec.getX()<<","<<tfVec.getY()<<","<<tfVec.getZ()<<endl;

    Quaterniond q(tf.getRotation());   
    cout<<"quaternion: " <<q.coeffs().transpose()<<endl;

    Matrix3d R_t=q.toRotationMatrix();
    cout<<"quaternion to rotation matrix: \n" <<R_t<<endl;
}

void tfListener::printStampedTf(tf::StampedTransform sTf)
{
    tf::Transform tf;

    cout<<"frame_id: "<<sTf.frame_id_<<endl;
    cout<<"child_frame_id: "<<sTf.child_frame_id_<<endl; 

    tf=get_tf_from_stamped_tf(sTf);
    printTf(tf);
}

geometry_msgs::PoseStamped tfListener::get_pose_from_transform(tf::StampedTransform tf)
{
    geometry_msgs::PoseStamped stPose;
    Vector3d p(tf.getOrigin());
    Quaterniond q(tf.getRotation());
    
    stPose.pose.position.x=p(0);stPose.pose.position.y=p(1);stPose.pose.position.z=p(2);
    stPose.pose.orientation.x=q.x();stPose.pose.orientation.y=q.y();
    stPose.pose.orientation.z=q.z();stPose.pose.orientation.w=q.w();
    stPose.header.frame_id = tf.frame_id_; 
    stPose.header.stamp = tf.stamp_; 
    return stPose;
}

void tfListener::printStampedPose(geometry_msgs::PoseStamped stPose)
{
    Vector3d p;
    p(0)=stPose.pose.position.x;p(1)=stPose.pose.position.y;p(2)=stPose.pose.position.z;
    Quaterniond q(stPose.pose.orientation.w,stPose.pose.orientation.x,stPose.pose.orientation.y,stPose.pose.orientation.z);    

    cout<<"frame id = "<<stPose.header.frame_id<<endl;
    cout<<"origin: "<<p.transpose()<<endl;
    cout<<"quaternion: "<<q.coeffs().transpose()<<endl;
}