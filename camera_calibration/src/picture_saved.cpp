#include <ros/ros.h>
#include <grasp_planning_msgs/GenerateGraspsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <ctime>

#include <cv.hpp>
#include <highgui.h>

using namespace std;
int num=0;
class PictureSaved
{
public:
    PictureSaved(ros::NodeHandle& nh):nh_(nh)
    {

        cout<<"begin save pictures!"<<endl;
        ros::Subscriber pic_sub = nh.subscribe("/xtion/rgb/image_raw", 1, & PictureSaved::PicSavedCB,this);
        ros::AsyncSpinner spinner(1);
        spinner.start();
        ros::spin();
    }
    void PicSavedCB(const sensor_msgs::Image msg);
private:
    ros::NodeHandle nh_;

};
void PictureSaved::PicSavedCB(const sensor_msgs::Image msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);

    char buff[100];
    sprintf(buff, "/home/weili/simple_kinova/src/camera_calibration/saved_image/saved_%d.png",num);
    num++;
    cout<<"======================saved picture num:"<<num<<"======================="<<endl;
    cout<<buff<<endl;
    cv::imwrite(buff, image_gray);

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_matrix_client");
    ros::NodeHandle nh;
    PictureSaved ps(nh);

    ROS_INFO("Finished");
    ros::spin();
    return 0;
}
