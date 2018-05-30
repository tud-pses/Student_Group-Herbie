#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <ctime>
#include <tf/tf.h>
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <math.h>

#include"MiniPID.h"


void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odo)
{
  *odo = *odomMsg;
}

void begin_cur_Callback(std_msgs::Bool::ConstPtr beginMsg, std_msgs::Bool* begin)
{
  *begin = *beginMsg;
}

void cur_type_Callback(std_msgs::Bool::ConstPtr typeMsg, std_msgs::Bool* type)
{
  *type = *typeMsg;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_corner_node");
    ros::NodeHandle nh;

      // some vars
      nav_msgs::Odometry odo;
      std_msgs::Int16 motor, steering;
      std_msgs::Bool curve_finished;
      std_msgs::Bool begin;
      std_msgs::Bool corner_type;

      // init subscribers
      ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
                  "/odom",10,boost::bind(odomCallback,_1,&odo));
      ros::Subscriber begin_cur =nh.subscribe<std_msgs::Bool>("begin_curve",10,boost::bind(begin_cur_Callback,_1,&begin));
      ros::Subscriber cur_type =nh.subscribe<std_msgs::Bool>("begin_curve",10,boost::bind(cur_type_Callback,_1,&corner_type));

      // generate control message publisher
      ros::Publisher motorCtrl =
          nh.advertise<std_msgs::Int16>("corner_motor_level", 1);
      ros::Publisher steeringCtrl =
          nh.advertise<std_msgs::Int16>("corner_steering_level", 1);
      ros::Publisher curve_finished_inf = nh.advertise<std_msgs::Bool>("simple_corner_finish_inf",1);

      ros::Rate loop_rate(25);

    enum turn{l,r};

	
    // set direction,angle,speed,parameters
    turn direction=r;
    double angle = 90;
    int speed=350;
    int steering_angle;
    int steering_angle_long;
    nh.getParam("/simple_corner_node/steering_angle_long", steering_angle_long);
    int steering_angle_short =150;
    nh.param("/simple_corner_node/steering_angle_short", steering_angle_short, steering_angle_short);
    int steering_angle_2_half_long =350;
    nh.param("/simple_corner_node/steering_angle_2_half_long", steering_angle_2_half_long, steering_angle_2_half_long);
    int steering_angle_2_half_short =450;
    nh.param("/simple_corner_node/steering_angle_2_half_short", steering_angle_2_half_short, steering_angle_2_half_short);
    int steering_angle_2_threshold = 30;
    nh.param("/simple_corner_node/steering_angle_2_threshold", steering_angle_2_threshold, steering_angle_2_threshold);
    double roll, pitch, yaw,degree,old_degree,difference,set_angle;
    double set_back_360 =0;
	
    //angle and steering_angle right/left adjustment
    if(direction==r){
     angle=-angle;
     steering_angle=-steering_angle;

    }

    curve_finished.data = false;


      while (ros::ok())
      {

        // init odometry
          tf::Quaternion q;
          tf::quaternionMsgToTF(odo.pose.pose.orientation, q);
          tf::Matrix3x3 mat(q);
          mat.getEulerYPR(yaw, pitch, roll);
           degree =(yaw*180/M_PI)+180;

        curve_finished.data = false;

       //set start orientation
       if(begin.data&& degree>=0){
           set_angle=degree;
           set_back_360=0;
        }

       // decide speed and steering
       if(corner_type.data){
           steering_angle = steering_angle_long;
           speed=500;
       }
       else{
           steering_angle = steering_angle_short;
           speed=400;
       }

       //360 to 0 jump degree
       if(old_degree-degree>=300){
        set_back_360=360;
       }

       if(old_degree-degree<=-300){
           set_back_360 = -360;
       }

        //degree difference
        difference=degree-set_angle-angle+set_back_360;

        if(abs(difference) < steering_angle_2_threshold){
            if(corner_type.data)
                steering_angle = steering_angle_2_half_long;
            else
                steering_angle = steering_angle_2_half_short;
        }

        //turn until 90 degree nearly reached
        if(abs(difference)>=4){
          steering.data = steering_angle;
           motor.data=speed;
        }else{
            curve_finished.data = true;
        }
		
        //set old_degree for 360 to 0 jump
        old_degree=degree;


        // publish command messages on their topics
        motorCtrl.publish(motor);
        steeringCtrl.publish(steering);
        curve_finished_inf.publish(curve_finished);

        ros::spinOnce();
        loop_rate.sleep();
      }

    }

