#include <ros/ros.h>
#include <std_msgs/Int16.h>


void v1Callback(std_msgs::Int16::ConstPtr objectMsg, std_msgs::Int16* object)
{
  *object = *objectMsg;
}

void v2Callback(std_msgs::Int16::ConstPtr objectMsg, std_msgs::Int16* object)
{
  *object = *objectMsg;
}

/**
 * @brief min
 *      returns the minimum of the given integers
 */
int min(int i, int j)
{
    if(i<j)
        return i;
    else return j;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vse_master_node");
    ros::NodeHandle nh;

    std_msgs::Int16 velocity, v1, v2;

    // subscribe to velocity topic of vse_node
    ros::Subscriber v1_sub = nh.subscribe<std_msgs::Int16>(
        "/vse_v_sign", 10, boost::bind(v1Callback, _1, &v1));

    // subscribe to velocity topic of vse_stop_node
    ros::Subscriber v2_sub = nh.subscribe<std_msgs::Int16>(
        "/vse_sign", 10, boost::bind(v2Callback, _1, &v2));

    ros::Publisher pub =
        nh.advertise<std_msgs::Int16>("vse_v", 1);

    ros::Rate loop_rate(10);

    ros::spinOnce();


   while (ros::ok()){

       // publish minimum of velocities
       velocity.data=min(v1.data,v2.data);
       pub.publish(velocity);

       loop_rate.sleep();
       ros::spinOnce();

   }
}
