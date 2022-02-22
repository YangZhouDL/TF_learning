#include "try_tf_listener_class.h"
using namespace std;

int main(int argc,char **argv)
{
    ros::init(argc, argv, "tryTfListener");
    ros::NodeHandle n_main;
    ROS_INFO("main: instantiating an object of type tfListener_class");
    tfListener demoTfListener(n_main);

    tf::StampedTransform stfBaseToLink2, stfBaseToLink1, stfLink1ToLink2;
    tf::StampedTransform testStfBaseToLink2;

    tf::Transform tfBaseToLink1, tfLink1ToLink2, tfBaseToLink2, altTfBaseToLink2;

    demoTfListener.tf_listener->lookupTransform("base_link", "link1", ros::Time(0), stfBaseToLink1);
    cout << endl << "base to link1: " << endl;
    demoTfListener.printStampedTf(stfBaseToLink1);
    tfBaseToLink1 = demoTfListener.get_tf_from_stamped_tf(stfBaseToLink1);

    demoTfListener.tf_listener->lookupTransform("link1", "link2", ros::Time(0), stfLink1ToLink2);
    cout << endl << "link1 to link2: " << endl;
    demoTfListener.printStampedTf(stfLink1ToLink2);
    tfLink1ToLink2 = demoTfListener.get_tf_from_stamped_tf(stfLink1ToLink2);

    demoTfListener.tf_listener->lookupTransform("base_link", "link2", ros::Time(0), stfBaseToLink2);
    cout << endl << "base to link2: " << endl;
    demoTfListener.printStampedTf(stfBaseToLink2);

    altTfBaseToLink2 = tfBaseToLink1*tfLink1ToLink2;
    cout << endl << "result of multiply tfBaseToLink1*tfLink1ToLink2: " << endl;
    demoTfListener.printTf(altTfBaseToLink2);

    if (demoTfListener.multiply_stamped_tfs(stfBaseToLink1, stfLink1ToLink2, testStfBaseToLink2)) 
    {
        cout << endl << "testStfBaseToLink2:" << endl;
        demoTfListener.printStampedTf(testStfBaseToLink2);
    }
    cout << endl << "attempt multiply of stamped transforms in wrong order:" << endl;
    demoTfListener.multiply_stamped_tfs(stfLink1ToLink2, stfBaseToLink1, testStfBaseToLink2);

    geometry_msgs::PoseStamped stPose, stPose_wrt_base;
    stPose = demoTfListener.get_pose_from_transform(stfLink1ToLink2);
    cout << endl << "pose link2 w/rt link1, from stfLink1ToLink2" << endl;
    demoTfListener.printStampedPose(stPose);

    demoTfListener.tf_listener->transformPose("base_link", stPose, stPose_wrt_base);
    cout << endl << "pose of link2 transformed to base frame:" << endl;
    demoTfListener.printStampedPose(stPose_wrt_base);

    return 0;
}