#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>


void idCallback(std_msgs::Float32::ConstPtr objectMsg, std_msgs::Float32* object)
{
  *object = *objectMsg;
}

void distanceCallback(std_msgs::Float32::ConstPtr objectMsg, std_msgs::Float32* object)
{
  *object = *objectMsg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vse_stop_node");
    ros::NodeHandle nh;

    // variables for detected sign id and distance
    std_msgs::Float32 id_, distance_;
    std_msgs::Int16 velocity;

    int state;
    float id,distance;

    // subscribe to find-object id and distance topics
    ros::Subscriber sub_id = nh.subscribe<std_msgs::Float32>(
        "/vse_id", 10, boost::bind(idCallback, _1, &id_));

    ros::Subscriber sub_distance = nh.subscribe<std_msgs::Float32>(
        "/vse_distance", 10, boost::bind(distanceCallback, _1, &distance_));

    ros::Publisher pub =
        nh.advertise<std_msgs::Int16>("vse_sign", 1);

    ros::Rate loop_rate(10);

    ros::spinOnce();

    // default velocity
    velocity.data = 1000;

    // offset for STOP sign id's
    int i=30;
    int j=i+10;


   while (ros::ok()){

       id=id_.data;
       distance=distance_.data;
    ROS_INFO("%d", state);

       switch(state){
       // no STOP sign detected
       case 0:{
           if(id>=i && id<j && distance<=5.0 && distance > 2.1)
               state = 1;
           velocity.data = 1000;
           break;
       }

       //STOP sign in range -> slow down
       case 1:{
           if(id>=i && id<j && distance<=2.0)
               state = 11;
            velocity.data = 200;
           break;
       }

       // STOP sign close -> wait 3s
       case 11:{
            velocity.data = 0;
            pub.publish(velocity);
            ros::Duration(3).sleep();
            state=0;
            break;
       }

       }

       pub.publish(velocity);
       loop_rate.sleep();
       ros::spinOnce();

   }
}
