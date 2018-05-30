#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/tf.h>


void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}

/**
 * @brief average
 *      calculates average of a 10 item arrays
 */
double average(double in[10]){
    double sum=0;
    int j=0;
    for(j;j<10;j++){
        sum+=in[j];
    }
    return sum/10;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "USR_Filter");
    ros::NodeHandle nh;

    sensor_msgs::Range usr, usr_filtered;

    double ranges[10];
    for(int i=0;i<10;i++){
        ranges[i]=0.75;
    }
    int count =0;
    ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
        "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));

    ros::Publisher USR_Publisher =
        nh.advertise<sensor_msgs::Range>("USR_Filtered", 1);

    ros::Rate loop_rate(25);

    while (ros::ok()){
        //wait for 10 proper values
        if(count<10)
            count++;

        // shift array
        int i=9;
        for(i;i>0;i--){
            ranges[i]=ranges[i-1];
        }

        // replace with old value if current is 0
        if(std::isnan(usr.range)||usr.range==0)
            ranges[0]=ranges[1];
        else
            ranges[0]=usr.range;

        // publish average of last 10 values
        usr_filtered.range=average(ranges);
        if(count>=10)
            USR_Publisher.publish(usr_filtered);
        else
           USR_Publisher.publish(usr);

        ros::spinOnce();
        loop_rate.sleep();
    }

}
