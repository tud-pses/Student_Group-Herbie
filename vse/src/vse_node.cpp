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
    ros::init(argc, argv, "vse_node");
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
        nh.advertise<std_msgs::Int16>("vse_v_sign", 1);

    ros::Rate loop_rate(10);

    ros::spinOnce();

    // default velocity
    velocity.data = 1000;

    // offset for different sign id's
    int i=10;
    int j=i+10;
    int k=j+10;

    // relevant detection range
    double min_range=3.5;


     while (ros::ok()){

       id=id_.data;
       distance=distance_.data;

       //0: nothing detected
       //1: 30 sign
       //2: 50 sign
       //3: 70 sign

       if(id<i && id>0 && distance<=min_range)
           state = 1;
       else if(id>=i && id<j && distance<=min_range)
           state = 2;
       else if(id>=j && id<k && distance<=min_range)
           state = 3;
       else state =0;

       switch(state){

           case 1:{
                velocity.data = 200;
               break;
           }
           case 2:{
                velocity.data = 300;
               break;
           }
           case 3:{
               velocity.data = 500;
               break;
           }
           case 0:{
               break;
           }
       }


       pub.publish(velocity);
       loop_rate.sleep();
       ros::spinOnce();

   }
}
