#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include<iostream>
#include<iomanip>
#include <math.h>
#include <ctime>
#include <chrono>

using namespace std;

void laserCallback(sensor_msgs::LaserScan::ConstPtr laserMsg, sensor_msgs::LaserScan* laser)
{
  *laser = *laserMsg;
}

void motor0Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void motor1Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void steering0Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void steering1Callback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

void cornerCallback(std_msgs::Bool::ConstPtr msg, std_msgs::Bool* retu)
{
  *retu = *msg;
}

void vseCallback(std_msgs::Int16::ConstPtr msg, std_msgs::Int16* retu)
{
  *retu = *msg;
}

std_msgs::Int16 min(std_msgs::Int16 i, std_msgs::Int16 j)
{
    if(i.data<j.data)
        return i;
    else return j;
}

//returns true if first 10 not nan numbers of laser.ranges are greater than 3
bool corner(sensor_msgs::LaserScan laser, double corner_range)
{
    if(laser.ranges.empty())
        return false;

  // ROS_INFO("%f", laser.range_max);
  // ROS_INFO("%f", laser.ranges[6]);
     int i=3;
    int j=0;
    while(j<=8){
        if(isnan(laser.ranges[i]))
            i++;
        else{
            if(laser.ranges[i]<corner_range)
                return false;
            i++;
            j++;
        }
    }


    return true;
}

//returns true if corner_type is long
bool long_corner(sensor_msgs::LaserScan laser)
{
    //int h=253; ist etwa Mitte
    int h=190;
   while(isnan(laser.ranges[h]))
       h++;
    if(laser.ranges[h]<3.0)
        return false;
    return true;
}

int main(int argc, char** argv)
{
    // init this node
    ros::init(argc, argv, "rundkurs_node");

    // get ros node handle
    ros::NodeHandle nh;
    // sensor message container
    sensor_msgs::LaserScan laser;
    std_msgs::Int16 motor0in;
    std_msgs::Int16 motor1in;
    std_msgs::Int16 motorvse;
    std_msgs::Int16 steering0in;
    std_msgs::Int16 steering1in;
    std_msgs::Bool corner_fin;
    std_msgs::Bool corner_start;
    std_msgs::Bool corner_type;


    bool vse = false;
    double corner_range= 3.0;
    nh.param("/rundkurs_node/vse", vse, vse);
    nh.param("/rundkurs_node/corner_range", corner_range, corner_range);


    // generate subscriber for sensor messages
    ros::Subscriber laserSub = nh.subscribe<sensor_msgs::LaserScan>( "/scan", 1, boost::bind(laserCallback, _1, &laser));
    ros::Subscriber motor0Sub = nh.subscribe<std_msgs::Int16>( "wall_motor_level", 1, boost::bind(motor0Callback, _1, &motor0in));
    ros::Subscriber motor1Sub = nh.subscribe<std_msgs::Int16>( "corner_motor_level", 1, boost::bind(motor1Callback, _1, &motor1in));
    ros::Subscriber steering0Sub = nh.subscribe<std_msgs::Int16>( "wall_steering_level", 1, boost::bind(steering0Callback, _1, &steering0in));
    ros::Subscriber steering1Sub = nh.subscribe<std_msgs::Int16>( "corner_steering_level", 1, boost::bind(steering1Callback, _1, &steering1in));
    ros::Subscriber cornerSub = nh.subscribe<std_msgs::Bool>( "simple_corner_finish_inf", 1, boost::bind(cornerCallback, _1, &corner_fin));
    ros::Subscriber vseSub;

    // activate vse if true
    if(vse)
        vseSub = nh.subscribe<std_msgs::Int16>( "vse_v", 1, boost::bind(vseCallback, _1, &motorvse));

    //generate publisher
    ros::Publisher motorCtrl =
        nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
    ros::Publisher steeringCtrl =
        nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
    ros::Publisher cornerStart =
        nh.advertise<std_msgs::Bool>("begin_curve", 1);
    ros::Publisher cornerType =
        nh.advertise<std_msgs::Bool>("corner_type", 1);

    // state 0 -> straight
    // state 1 -> corner
    int state = 0;

    // initialize clock for preventing missdetections
    auto start = std::chrono::system_clock::now();
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds;

    ros::Rate loop_rate(25);
    ros::Duration(6).sleep();

    while (ros::ok()){

        switch(state){

            case 0:{

                if(vse)
                    motorCtrl.publish(min(motor0in,motorvse));
                else
                    motorCtrl.publish(motor0in);
                    steeringCtrl.publish(steering0in);
                    corner_start.data = true;
                    cornerStart.publish(corner_start);
                    end = std::chrono::system_clock::now();
                    elapsed_seconds = end - start;
                if(corner(laser, corner_range) &&  elapsed_seconds.count() > 0.7){
                    corner_start.data = false;
                    cornerStart.publish(corner_start);
                    corner_type.data=long_corner(laser);
                    ROS_INFO("Ecke: %d", corner_type.data);
                    cornerType.publish(corner_type);
                    state=1;
                    ROS_INFO("corner detected");
                }
                break;
            }

            case 1:{
                if(vse)
                    motorCtrl.publish(min(motor1in,motorvse));
                else
                    motorCtrl.publish(motor1in);
                steeringCtrl.publish(steering1in);
                if(corner_fin.data){
                    start = std::chrono::system_clock::now();
                    state=0;
                    ROS_INFO("corner finished________________");
                 }
                break;
            }
        }



   loop_rate.sleep();
   ros::spinOnce();
    }
    ros::spin();
//return 0;
}
