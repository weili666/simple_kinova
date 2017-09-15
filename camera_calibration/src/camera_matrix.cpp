#include <ros/ros.h>
#include <grasp_planning_msgs/GenerateGraspsAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <ctime>

#include <cv.hpp>
#include <highgui.h>

#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/eigen.hpp>
#include <algorithm>

using namespace std;
int num=0;
class CameraMatrixCalib
{
public:
    CameraMatrixCalib(ros::NodeHandle& nh,int num_cols,int num_rows,double dist):nh_(nh)
    {
        cols = num_cols;
        rows = num_rows;
        distance = dist;
        point_counts=cols*rows;
        cv::Size patternSize(cols,rows);
        for (int i=0;i<cols;i++)
        {
            for (int j=0;j<rows;j++)
            {
                worldPoints.push_back(cv::Point3f(i*distance,j*distance,0));
            }
        }

        cv::Mat img;
        cv::Mat calpic;
        cv::Mat img2;
        char buff[100];
        int i=0;

        //===========================DETECT CORNER============================//
        for(i=0;i<17;i++)
        {
            sprintf(buff, "/home/weili/simple_kinova/src/camera_calibration/saved_image/saved_%d.png",i);
            cout<<buff<<endl;
            img = cv::imread(buff);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);
            cout<<"haha"<<endl;
            bool find=cv::findChessboardCorners(img,patternSize,corners);
            cout<<"haha1"<<endl;

//            cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
//                        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            cornersVect.push_back(corners);
            worldPointsVect.push_back(worldPoints);

            cv::drawChessboardCorners(img,patternSize,corners,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/simple_kinova/src/camera_calibration/saved_image/draw_chessboard_coners_%d.png",i);
            cv::imwrite(buff2, img);
        }
        //===================================================================//

        cv::calibrateCamera(worldPointsVect,cornersVect,calpic.size(),cameraMatrix,distCoeffs,rvecs,tvecs);

        //============================RE-PROJECT=============================//
        for(int i=0;i<cornersVect.size();i++)
        {
            sprintf(buff, "/home/weili/simple_kinova/src/camera_calibration/saved_image/saved_%d.png",i);
            cout<<buff<<endl;
            img = cv::imread(buff);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(worldPointsVect[i],rvecs[i],tvecs[i],cameraMatrix,distCoeffs,projectedPoints);
            cornersVectProj.push_back(projectedPoints);

            cv::drawChessboardCorners(calpic,patternSize,projectedPoints,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/simple_kinova/src/camera_calibration/saved_image/draw_chessboard_project_coners_%d.png",i);
            cv::imwrite(buff2, calpic);
        }
        //===================================================================//

        cout<<"<<<<<<<<<<<<<---------calpic_size-------->>>>>>>>>>>>>>>"<<endl;
        cout<<calpic.size()<<endl;
        cv::Rodrigues(rvecs[rvecs.size()-1],rotMatrix,JacoMatrix);
        cout<<"size of connersVect:"<<cornersVect.size()<<endl;
        cout<<"------------------------cameraMatrix------------------"<<endl;
        cout<<cameraMatrix<<endl;
        cout<<"------------------------distCoeffs--------------------"<<endl;
        cout<<distCoeffs<<endl;
        cout<<"------------------------rvecs-------------------------"<<endl;
        cout<<rvecs[rvecs.size()-1]<<endl;
        cout<<"------------------------rotMatrix---------------------"<<endl;
        cout<<rotMatrix<<endl;
        cout<<"------------------------JacoMatrix--------------------"<<endl;
        cout<<JacoMatrix<<endl;
        cout<<"------------------------tvecs-------------------------"<<endl;
        cout<<tvecs[tvecs.size()-1]<<endl;

        cv::Mat tvec_mat = tvecs[tvecs.size()-1];


        ofstream file;
        file.open("/home/weili/simple_kinova/src/camera_calibration/saved_image/extern_matrix.txt",ios::app|ios::out);
        file<<rotMatrix<<endl;
        file<<tvec_mat<<endl;



        cv::waitKey(10);

        system("pause");
}
 private:
    ros::NodeHandle nh_;
    int cols;
    int rows;
    float distance;    //间距30mm
    std::vector<cv::Point2f> corners;
    std::vector<std::vector<cv::Point2f> > cornersVect;
    std::vector<std::vector<cv::Point2f> > cornersVectProj;
    std::vector<cv::Point3f> worldPoints;
    std::vector<std::vector<cv::Point3f> > worldPointsVect;
    int point_counts;
    cv::Mat cameraMatrix,distCoeffs;
    cv::Mat rotMatrix,JacoMatrix;
    std::vector<cv::Mat> rvecs,tvecs;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_matrix_client");
    ros::NodeHandle nh;
    int num_cols = 12;
    int num_rows = 12;
    double dist = 20.286;
    CameraMatrixCalib cmc(nh,num_cols,num_rows,dist);

    ROS_INFO("Finished");
    ros::spin();
    return 0;
}
