/**
  *Source: https://github.com/wennycooper/learning_nav/blob/master/src/mybot_laser_obstacle_clearing_filter.cpp
  *
  *
*/
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <math.h>
#include <array>
using namespace std;


void laserCallback(sensor_msgs::LaserScan::ConstPtr laserMsg, sensor_msgs::LaserScan* laser)
{
  *laser = *laserMsg;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "LaserScan_Filter");
    ros::NodeHandle nh;

    sensor_msgs::LaserScan filtered_scan, input_scan;
    ros::Subscriber input_scan_sub = nh.subscribe<sensor_msgs::LaserScan>( "/scan_old", 1, boost::bind(laserCallback, _1, &input_scan));
    ros::Publisher filtered_scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
    ros::spinOnce();
    ros::Rate loop_rate(25);

    ros::spinOnce();

    while(ros::ok()){

        if(input_scan.ranges.size()>0){
            //check whether the laserscan contains inf or nan values set them to max value
            filtered_scan.ranges.resize(input_scan.ranges.size());
            for (unsigned int i = 0; i < input_scan.ranges.size(); ++i){
                if(input_scan.ranges[i] > input_scan.range_max || isnan(input_scan.ranges[i]) || isinf(input_scan.ranges[i])) {
                    filtered_scan.ranges[i] = input_scan.range_max;
                }
                else {
                    filtered_scan.ranges[i] = input_scan.ranges[i];
                }
            }

            //set the first 3 to a value that is not a nan value
            if(!isnan(input_scan.ranges[3])){
                filtered_scan.ranges[0]=input_scan.ranges[3];
                filtered_scan.ranges[1]=input_scan.ranges[3];
                filtered_scan.ranges[2]=input_scan.ranges[3];
            }

            //set the last 3 values to a value that is not a nan value
            if(!isnan(input_scan.ranges[input_scan.ranges.size()-4])){
              filtered_scan.ranges[filtered_scan.ranges.size()-1]=input_scan.ranges[input_scan.ranges.size()-4];
              filtered_scan.ranges[filtered_scan.ranges.size()-2]=input_scan.ranges[input_scan.ranges.size()-4];
              filtered_scan.ranges[filtered_scan.ranges.size()-3]=input_scan.ranges[input_scan.ranges.size()-4];
            }

            filtered_scan.header.frame_id = input_scan.header.frame_id;
            filtered_scan.header.stamp = input_scan.header.stamp;
            filtered_scan.angle_min = input_scan.angle_min;
            filtered_scan.angle_max = input_scan.angle_max;
            filtered_scan.angle_increment = input_scan.angle_increment;
            filtered_scan.time_increment = input_scan.time_increment;
            filtered_scan.scan_time = input_scan.scan_time;
            filtered_scan.range_min = input_scan.range_min;
            filtered_scan.range_max = input_scan.range_max;

            filtered_scan_pub.publish(filtered_scan);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}
