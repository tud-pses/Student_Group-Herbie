#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>


#include <tf/tf.h>
#include <nav_msgs/Odometry.h>



using namespace std;


// gets called whenever a new message is availible in the input buffer
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input buffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input buffer
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}


void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odo)
{
  *odo = *odomMsg;
}


// gets called whenever a new message is availible in the input buffer
void twistCallback(geometry_msgs::Twist::ConstPtr twistMsg, geometry_msgs::Twist* twist)
{
    *twist = *twistMsg;
}



int main(int argc, char** argv)
{


  // init this node
  ros::init(argc, argv, "base_controller");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  sensor_msgs::Range usr, usf, usl;
  std_msgs::Int16 motor, steering;
  nav_msgs::Odometry odo;
  geometry_msgs::Twist twist;



  // generate subscriber for sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));

  ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
              "/odom",10,boost::bind(odomCallback,_1,&odo));

  ros::Subscriber twistSub = nh.subscribe<geometry_msgs::Twist>(
              "/cmd_vel",10,boost::bind(twistCallback,_1,&twist));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  double twist_linear = 625;
  nh.param("/base_controller/twist_linear", twist_linear, twist_linear);

  double twist_angular = 1000;
  nh.param("/base_controller/twist_angular", twist_angular, twist_angular);


  ros::Rate loop_rate(25);

  double steering_angle = 0;

  while (ros::ok())
   {
      if(twist.linear.x == 0){
        steering_angle =0;
      }else{
        //twist message contains angle
        steering_angle = twist.angular.z;
      }
        motor.data = twist.linear.x*twist_linear;
        steering.data = -(twist.angular.z*twist_angular);

        if(steering.data > 800){
          steering.data = 800;
        }else if(steering.data < (-800)){
          steering.data = -800;
        }


        // publish command messages on their topics
        motorCtrl.publish(motor);
        steeringCtrl.publish(steering);

        // clear input/output buffers
        ros::spinOnce();
        // this is needed to ensure a const. loop rate
          loop_rate.sleep();
   }

  ros::spin();
}

