#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>

class ImageCutNodelet
{

    image_transport::Publisher img_pub_;
    image_transport::Subscriber img_sub_;

    image_transport::Publisher img_pub_depth;
    image_transport::Subscriber img_sub_depth;

/*
    const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
    {
    if (frame.empty())
      return image_frame;
    return frame;
    }
*/
    void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
    do_work(msg, cam_info->header.frame_id);
    }

    void depthImageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
    do_work_depth(msg, cam_info->header.frame_id);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
    do_work(msg, msg->header.frame_id);
    }

    void depthImageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
    do_work_depth(msg, msg->header.frame_id);
    }


    /**
     * @brief do_work
     *      cuts the input color image to the relevant area
     * @param msg
     *      input color image
     * @param input_frame_from_msg
     *      camera info
     */
    void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
    {
    // Work on the image.

      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::Mat out_image(in_image ,cv::Range(in_image.rows/3, in_image.rows*2/3), cv::Range(0, in_image.cols/2));

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", out_image).toImageMsg();
      img_pub_.publish(out_img);


    }

    /**
     * @brief do_work_depth
     *      cuts the input depth image to the relevant area
     * @param msg
     *      input depth image
     * @param input_frame_from_msg
     *      camera info
     */
    void do_work_depth(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
    {
    // Work on the image.

      // Convert the image into something opencv can handle.
      cv::Mat in_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
      cv::Mat out_image(in_image ,cv::Range(in_image.rows/3, in_image.rows*2/3), cv::Range(0, in_image.cols/2));


      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "16UC1", out_image).toImageMsg();
      img_pub_depth.publish(out_img);

    }


    public:

    ImageCutNodelet(image_transport::ImageTransport it){
        this->init(it);
    }

    /**
     * @brief init
     *      initializes publishers and subscribers of the ImageCutNodelet
     * @param it_
     *      image_transport
     */
    void init(image_transport::ImageTransport it_)
    {
        //init subscribers
        img_sub_ = it_.subscribe("image", 3, &ImageCutNodelet::imageCallback, this);
        img_sub_depth = it_.subscribe("depthimage", 3, &ImageCutNodelet::depthImageCallback, this);

        //init publishers
        img_pub_ = it_.advertise("/image_cut", 1);
        img_pub_depth = it_.advertise("/depthimage_cut", 1);

  }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "image_cut_node");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it(nh_);
    ImageCutNodelet img_cut(it);
    ros::spin();

}
