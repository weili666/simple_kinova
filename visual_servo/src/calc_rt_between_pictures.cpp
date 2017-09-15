#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <ctime>
#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/nonfree/nonfree.hpp>

#include <fstream>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
//joint state msg related
#include <wpi_jaco_msgs/JointPosiCartForce.h>
#include <std_msgs/Float32MultiArray.h>
//KDL related
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

//boost related
#include <boost/scoped_ptr.hpp>
#include <boost/make_shared.hpp>

//torque command message related
#include <wpi_jaco_msgs/AngularTorqueCommand.h>
#include <wpi_jaco_msgs/CartesianForceCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/StartForceControl.h>
#include <wpi_jaco_msgs/StopForceControl.h>
#include <wpi_jaco_msgs/SetFingersPositionAction.h>

//boost
#include <boost/shared_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
using namespace std;
using namespace cv;

double rand01()
{
   // srand(time(NULL));
    return (rand()/((float)RAND_MAX));
}

class ArmJoints
{
public:
    ArmJoints()
    {
        joint1=0;joint2=M_PI;joint3=M_PI;joint4=0;joint5=0;joint6=0;
    }
    ArmJoints(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6):joint1(jo1),joint2(jo2),joint3(jo3),joint4(jo4),joint5(jo5),joint6(jo6){}
    ArmJoints(const ArmJoints& aj)
    {
        joint1=aj.joint1;joint2=aj.joint2;joint3=aj.joint3;joint4=aj.joint4;joint5=aj.joint5;joint6=aj.joint6;
    }
    void setJoint(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6)
    {
        joint1=jo1;joint2=jo2;joint3=jo3;joint4=jo4;joint5=jo5;joint6=jo6;
    }
    double joint1,joint2,joint3,joint4,joint5,joint6;

};
class CalcRtBetweenPics
{
public:
    CalcRtBetweenPics(cv::Mat* pic_1, cv::Mat* pic_2, cv::Mat* cameraMatrix_, cv::Mat* distCoeffs_, cv::Mat* handeyeMatrix_)
    {

        picture1_c = pic_1->clone();
        picture2_c = pic_2->clone();
        cameraMatrix = cameraMatrix_->clone();
        distCoeffs = distCoeffs_->clone();
        handeyeMatrix = handeyeMatrix_->clone();


        //===============feature detect and descripte===============//

        initModule_nonfree();
        Ptr<FeatureDetector> detector = FeatureDetector::create( "SIFT" );
        Ptr<DescriptorExtractor> descriptor_extractor = DescriptorExtractor::create( "SIFT" );
        struct timeval start_t, end_t;
        gettimeofday( &start_t, NULL );
        double start = getTickCount();cout<<"hello world!"<<endl;
        detector->detect(picture1_c,kp1);
        detector->detect(picture2_c,kp2);
        descriptor_extractor->compute(picture1_c,kp1,des1);
        descriptor_extractor->compute(picture2_c,kp2,des2);
        int drawmode = DrawMatchesFlags::DRAW_RICH_KEYPOINTS;
        drawKeypoints(picture1_c,kp1,res1,Scalar::all(-1),drawmode);
        drawKeypoints(picture2_c,kp2,res2,Scalar::all(-1),drawmode);
        cout<<"size of  Img1: "<<picture1_c.size()<<" , "<<picture1.size()<<endl;
        cout<<"size of  Img2: "<<picture2_c.size()<<" , "<<picture2.size()<<endl;

        //==========================================================//

        //================match key points between two pictures==============//

        Ptr<DescriptorMatcher> descriptor_matcher = DescriptorMatcher::create( "BruteForce" );
        descriptor_matcher->match(des1,des2,matches);
        double end = getTickCount();
        gettimeofday( &end_t, NULL );
        cout<<"耗时："<<(end - start) <<"ns"<<endl;
        cout<<"耗时："<<(end_t.tv_usec - start_t.tv_usec) <<"us"<<endl;

        double max_dst = 0;
        double min_dst = 100;
        for(int i = 0; i < matches.size(); i++)
        {
            double dist = matches[i].distance;
            if(dist < min_dst) min_dst = dist;
            if(dist > max_dst) max_dst = dist;
        }
        cout<<"min_distance:"<<min_dst<<",max_distance:"<<max_dst<<endl;


        for(int i = 0; i < matches.size(); i++)
        {
            if(matches[i].distance < min_dst+0.2*(max_dst - min_dst))
            {
                good_matches.push_back(matches[i]);
            }
        }

        cout<<"number of matched points:"<<matches.size()<<";number of good_matched points: "<<good_matches.size()<<endl;

        //===================================================================//

        vector<Point2f> srcPoints(good_matches.size());
        vector<Point2f> dstPoints(good_matches.size());
        for (size_t i = 0; i < good_matches.size(); i++) {
            srcPoints[i] = kp1[good_matches[i].queryIdx].pt;
            dstPoints[i] = kp2[good_matches[i].trainIdx].pt;
        }


        //=================calculate the homography matrix=================//

         double reprojectionThreshold = 3;
         cv::Mat homography;

         cout<<"hello world !!"<<endl;
         homography = cv::findHomography(srcPoints, dstPoints, RANSAC, reprojectionThreshold);
         cout<<"=================homography matrix:================"<<endl;
         cout<<homography<<endl;


         //得到image_1的角点(需要寻找的物体)//////////
         std::vector<Point2f> obj_corners(4);
         obj_corners[0] = cvPoint(0,0);
         obj_corners[1] = cvPoint(0,picture1_c.rows);
         obj_corners[2] = cvPoint(picture1_c.cols,picture1_c.rows);
         obj_corners[3] = cvPoint(picture1_c.cols,0);
         std::vector<Point2f> scene_corners(4);

         //匹配四个角点/////////////////////////////////////
         perspectiveTransform(obj_corners, scene_corners, homography);

         cout<<"origin corners:"<<obj_corners[0]<<","<<obj_corners[1]<<","<<obj_corners[2]<<","<<obj_corners[3]<<endl;
         cout<<"perspective trasform corners:"<<scene_corners[0]<<","<<scene_corners[1]<<","<<scene_corners[2]<<","<<scene_corners[3]<<endl;

         //===================================================================//

         //=========================calculate PnP===========================//

         Mat He2o_(4, 4, CV_64FC1);
         Mat Ho2o_(4, 4, CV_64FC1);

         Mat r(3, 1, CV_64FC1);
         Mat t(3, 1, CV_64FC1);
         vector<Point3f> pts_3d(4);
         double dx = 0.063;//the width of the object;
         double dy = 0.113;//the height of the object;
         double dz = 0.110;//the length of the grasp;
         pts_3d[0] = Point3f(0,0,0);
         pts_3d[1] = Point3f(0,dy,0);//Point3f(0.113,0,0);
         pts_3d[2] = Point3f(dx,dy,0);//Point3f(0.113,0.063,0);
         pts_3d[3] = Point3f(dx,0,0);//Point3f(0,0.063,0);
         solvePnP(pts_3d,scene_corners,cameraMatrix,distCoeffs,r,t,false);
         Mat R(3, 3, CV_64FC1);
         Mat handeyeMatrix_R = handeyeMatrix.clone();
         handeyeMatrix_R.at<double>(0,3) = 0;
         handeyeMatrix_R.at<double>(1,3) = 0;
         handeyeMatrix_R.at<double>(2,3) = 0;
         Mat handeyeMatrix_inv_R(4, 4, CV_64FC1);
         handeyeMatrix_inv_R = handeyeMatrix_R.inv();

         cv::Rodrigues(r,R);

         cout<<"===========================PNP_RESULT=========================="<<endl;
         cout<<"r:"<<r<<endl<<"R:"<<R<<endl<<"t:"<<t<<endl;
         R.copyTo(He2o_(Rect(0, 0, 3, 3)));
         t.copyTo(He2o_(Rect(3, 0, 1, 3)));
         He2o_.at<double>(3, 0) = 0.0;
         He2o_.at<double>(3, 1) = 0.0;
         He2o_.at<double>(3, 2) = 0.0;
         He2o_.at<double>(3, 3) = 1.0;

         Ho2o_.at<double>(0, 0) = 1.0;
         Ho2o_.at<double>(0, 1) = 0;
         Ho2o_.at<double>(0, 2) = 0;
         Ho2o_.at<double>(0, 3) = dx/2;
         Ho2o_.at<double>(1, 0) = 0;
         Ho2o_.at<double>(1, 1) = 1.0;
         Ho2o_.at<double>(1, 2) = 0;
         Ho2o_.at<double>(1, 3) = dx/3;
         Ho2o_.at<double>(2, 0) = 0;
         Ho2o_.at<double>(2, 1) = 0;
         Ho2o_.at<double>(2, 2) = 1.0;
         Ho2o_.at<double>(2, 3) = -dz;
         Ho2o_.at<double>(3, 0) = 0;
         Ho2o_.at<double>(3, 1) = 0;
         Ho2o_.at<double>(3, 2) = 0;
         Ho2o_.at<double>(3, 3) = 1.0;
         hand2objMatrix.create(4, 4, CV_64FC1);
         hand2objMatrix = handeyeMatrix*He2o_*Ho2o_*handeyeMatrix_inv_R;
         cout<<endl<<"===========================Hand_To_Object_Matrix=========================="<<endl;
         cout<<hand2objMatrix<<endl;

         //=================================================================//

//         cv::Mat img_match2;
//         drawMatches(picture1_c, kp1, picture2_c, kp2, good_matches, img_match2);



//         line(img_match2, obj_corners[0] , obj_corners[1] , Scalar(0,255,0), 4);
//         line(img_match2, obj_corners[1] , obj_corners[2] , Scalar(0,255,0), 4);
//         line(img_match2, obj_corners[2] , obj_corners[3] , Scalar(0,255,0), 4);
//         line(img_match2, obj_corners[3] , obj_corners[0] , Scalar(0,255,0), 4);

//         line(img_match2, scene_corners[0] + Point2f(picture1_c.cols, 0), scene_corners[1] + Point2f(picture1_c.cols, 0), Scalar(0,255,0), 4);
//         line(img_match2, scene_corners[1] + Point2f(picture1_c.cols, 0), scene_corners[2] + Point2f(picture1_c.cols, 0), Scalar(0,255,0), 4);
//         line(img_match2, scene_corners[2] + Point2f(picture1_c.cols, 0), scene_corners[3] + Point2f(picture1_c.cols, 0), Scalar(0,255,0), 4);
//         line(img_match2, scene_corners[3] + Point2f(picture1_c.cols, 0), scene_corners[0] + Point2f(picture1_c.cols, 0), Scalar(0,255,0), 4);

//         char buff[100];
//         sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/servo_image/saved_%d.png",3);
//         cv::imwrite(buff, img_match2);
//         cout<<"image saved"<<endl;
//         sleep(5);
    }
    void getHand2ObjMatrix( cv::Mat* mat );

private:
    cv::Mat picture1_c,picture2_c;
    cv::Mat picture1,picture2;
    cv::Mat cameraMatrix,distCoeffs;
    cv::Mat handeyeMatrix;
    cv::Mat hand2objMatrix;

    vector<KeyPoint> kp1,kp2;
    cv::Mat des1,des2;
    cv::Mat res1,res2;
    vector<DMatch> matches;
    vector<DMatch> good_matches;
    vector<DMatch> inliers;
};

void CalcRtBetweenPics::getHand2ObjMatrix( cv::Mat* mat )
{
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            mat->at<double>(i,j) = hand2objMatrix.at<double>(i,j);
        }
    }
}

class PictureSaved
{
public:
    PictureSaved(ros::NodeHandle& nh):nh_(nh)
    {

        cout<<"begin save picture: "<<endl;
        ros::Subscriber pic_sub = nh_.subscribe("/camera/image_color", 1, & PictureSaved::PicSavedCB,this);

        ros::Rate loop_rate(1);
        int num_loop = 1;
        while(num_loop)
        {
            ros::spinOnce();
            loop_rate.sleep();
            num_loop--;
        }
    }
    void PicSavedCB(const sensor_msgs::Image& msg);
private:
    ros::NodeHandle nh_;

};
void PictureSaved::PicSavedCB(const sensor_msgs::Image& msg)
{

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::InputArray image=cv_ptr->image;

    char buff[100];
    sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/servo_image/saved_%d.png",2);

    cout<<"======================saved picture num:"<<2<<"======================="<<endl;
    cout<<buff<<endl;
    cv::imwrite(buff, image);

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_control");
    ros::NodeHandle nh;


    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    //ros::Rate loop_rate(10);

    actionlib::SimpleActionClient<wpi_jaco_msgs::SetFingersPositionAction>     finger_client_("jaco_fingers/finger_positions",true);
    wpi_jaco_msgs::SetFingersPositionGoal goal_f;

    planning_scene_monitor::PlanningSceneMonitorPtr scene_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();
    cout<<"scene set !"<<endl;
    moveit::planning_interface::MoveGroup robot_arm("arm");
    //moveit::planning_interface::MoveGroup robot_gripper("gripper");

    //=========================set kdl tree and group========================//
    KDL::Tree kdl_tree;
    cout<<"hello world!!"<<endl;

    string robot_desc_string;
    nh.param("robot_description",robot_desc_string,string());
    if(!kdl_parser::treeFromString(robot_desc_string,kdl_tree))
    {
        cout<<"Failed to construct kdl tree"<<endl;
    }

    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_;
    //boost::shared_ptr<KDL::ChainIdSolver>        pose_to_jnt_solver;

    KDL::Chain jaco_chain;
    if(!kdl_tree.getChain("support_link", "jaco_link_hand", jaco_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(jaco_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;
    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;

    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));
   // pose_to_jnt_solver.reset(new KDL::ChainIdSolver(jaco_chain));



    KDL::JntArray  q_;
    KDL::Frame     x_;
    KDL::Jacobian  J_;

    double a,b,c,d;
    q_.resize(6);
    J_.resize(jaco_chain.getNrOfJoints());

    //==========================================================================//

    //=====================grasp plan and public======================//

    finger_client_.waitForServer();
    goal_f.fingers.finger1 = 0;
    goal_f.fingers.finger2 = 0;
    goal_f.fingers.finger3 = 0;
    finger_client_.sendGoal(goal_f);
    bool finish_before_timeout0 = finger_client_.waitForResult(ros::Duration(2.0));

    //==========================================================================//

    //======================== move first to object =====================//

    ArmJoints arm(0.04138740419735123, 2.5852689545166005, 1.4389341664773672, -2.090824868635835, 1.445846667402777, 1.322086908385705);
    q_(0) = arm.joint1; q_(1) = arm.joint2; q_(2) = arm.joint3;
    q_(3) = arm.joint4; q_(4) = arm.joint5; q_(5) = arm.joint6;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    jnt_to_jac_solver_->JntToJac(q_,J_);
    cout<<"Jacobi Matrix:"<<J_.data<<endl;

    x_.M.GetQuaternion(a,b,c,d);
    cout<<"data M:"<<x_.M.data[0]<<" "<<x_.M.data[1]<<" "<<x_.M.data[2]<<" "<<endl<<x_.M.data[3]<<" "<<x_.M.data[4]<<" "<<x_.M.data[5]<<" "<<endl<<x_.M.data[6]<<" "<<x_.M.data[7]<<" "<<x_.M.data[8]<<endl;
    Mat grasp1(4, 4, CV_64FC1);
    Mat grasp2(4, 4, CV_64FC1);
    double m11,m12,m13,m21,m22,m23,m31,m32,m33;
    m11 = x_.M.data[0]; m12 = x_.M.data[1]; m13 = x_.M.data[2];
    m21 = x_.M.data[3]; m22 = x_.M.data[4]; m23 = x_.M.data[5];
    m31 = x_.M.data[6]; m32 = x_.M.data[7]; m33 = x_.M.data[8];
    grasp1.at<double>(0,0) = m11;
    grasp1.at<double>(0,1) = m12;
    grasp1.at<double>(0,2) = m13;
    grasp1.at<double>(1,0) = m21;
    grasp1.at<double>(1,1) = m22;
    grasp1.at<double>(1,2) = m23;
    grasp1.at<double>(2,0) = m31;
    grasp1.at<double>(2,1) = m32;
    grasp1.at<double>(2,2) = m33;
    grasp1.at<double>(0,3) = x_.p.x();
    grasp1.at<double>(1,3) = x_.p.y();
    grasp1.at<double>(2,3) = x_.p.z();
    grasp1.at<double>(3,0) = 0;
    grasp1.at<double>(3,1) = 0;
    grasp1.at<double>(3,2) = 0;
    grasp1.at<double>(3,3) = 1.0;


    std::map<std::string,double> joint;
    joint["jaco_joint_1"] = arm.joint1;
    joint["jaco_joint_2"] = arm.joint2;
    joint["jaco_joint_3"] = arm.joint3;
    joint["jaco_joint_4"] = arm.joint4;
    joint["jaco_joint_5"] = arm.joint5;
    joint["jaco_joint_6"] = arm.joint6;


    geometry_msgs::Pose target_pose0;
    target_pose0.orientation.x = a;
    target_pose0.orientation.y = b;
    target_pose0.orientation.z = c;
    target_pose0.orientation.w = d;
    target_pose0.position.x = x_.p.x();
    target_pose0.position.y = x_.p.y();
    target_pose0.position.z = x_.p.z();



    moveit::planning_interface::MoveGroup::Plan my_plan;
    robot_arm.setJointValueTarget(joint);
    //robot_arm.setPoseTarget(target_pose0);
    robot_arm.setPlanningTime(6.0);
    robot_arm.setPlannerId("RRTstarkConfigDefault");
    bool success_=robot_arm.plan(my_plan);
    sleep(3.0);
    robot_arm.execute(my_plan);
    sleep(3.0);
    PictureSaved ps(nh);
    sleep(2.0);


    robot_state::RobotState start_state(*robot_arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(robot_arm.getName());


    bool success_ik0 = start_state.setFromIK(joint_model_group, target_pose0);
    cout<<"0th stage find grasp pose:set from ik is success?:"<<success_ik0<<endl;
    if(success_ik0)
    {
       std::vector<double> joint_values;
       start_state.copyJointGroupPositions(joint_model_group, joint_values);
       const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
       for(std::size_t i = 0; i < joint_names.size(); ++i)
       {
           ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
       }
    }

    //==========================================================================//


    cv::Mat img,img2;
    //cv::Mat calpic,calpic2;
    char buff[100];
    char buff2[100];


    //===========================INPUT CAMERA MATRIX============================//

        sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/servo_image/saved_%d.png",1);
        cout<<buff<<endl;
        img = cv::imread(buff);
 //        cv::cvtColor(img, calpic, CV_BGR2GRAY);

        sprintf(buff2, "/home/weili/simple_kinova/src/visual_servo/servo_image/saved_%d.png",2);
        cout<<buff2<<endl;
        img2 = cv::imread(buff2);
 //        cv::cvtColor(img2, calpic2, CV_BGR2GRAY);


        ifstream ifile;
        ifile.open("/home/weili/simple_kinova/src/visual_servo/saved_image/extern_matrix.txt");
        char buffer[256];

        cout<<"hello world 1!!"<<endl;
        int num=0;
        int num_of_ros = 8;
        vector< vector<char> > data_;
        vector<char> data_iter;
        if(!ifile.is_open())
        {cout << "Error opening file"; exit (1);}
        while(num_of_ros)
        {
            cout<<"hello world 2!!"<<endl;
            data_iter.clear();
            ifile.getline(buffer,200,'\n');
            for(int i=0;i<256;i++)
            {
                if((buffer[i]=='[')||(buffer[i]==' '))
                {
                    continue;
                }
                else if(buffer[i]==',')
                {
                    data_.push_back(data_iter);
                    data_iter.clear();
                    num++;
                    continue;
                }
                else if(buffer[i]==';')
                {
                    data_.push_back(data_iter);
                    data_iter.clear();
                    num++;
                    break;
                }
                else if(buffer[i]==']')
                {
                    data_.push_back(data_iter);
                    data_iter.clear();
                    num++;
                    break;
                }
                else
                {
                    data_iter.push_back(buffer[i]);
                }
            }
            num_of_ros--;
        }
        double ddata[30];
        for(int i=0;i<30;i++)
        {
            string data;

            for(int j=0;j<data_[i].size();j++)
            {
                data+=data_[i][j];
            }
            ddata[i]=atof(data.c_str());
            cout<<"ddata"<<i<<":"<<ddata[i]<<endl;
        }
        cv::Mat cameraMatrix(3,3,CV_64FC1);
        cameraMatrix.at<double>(0,0) = ddata[0];
        cameraMatrix.at<double>(0,1) = ddata[1];
        cameraMatrix.at<double>(0,2) = ddata[2];
        cameraMatrix.at<double>(1,0) = ddata[3];
        cameraMatrix.at<double>(1,1) = ddata[4];
        cameraMatrix.at<double>(1,2) = ddata[5];
        cameraMatrix.at<double>(2,0) = ddata[6];
        cameraMatrix.at<double>(2,1) = ddata[7];
        cameraMatrix.at<double>(2,2) = ddata[8];
        cv::Mat distCoeffs(1,5,CV_64F);
        distCoeffs.at<double>(0,0) = ddata[9];
        distCoeffs.at<double>(0,1) = ddata[10];
        distCoeffs.at<double>(0,2) = ddata[11];
        distCoeffs.at<double>(0,3) = ddata[12];
        distCoeffs.at<double>(0,4) = ddata[13];
        cv::Mat handeyeMatrix(4,4,CV_64FC1);
        handeyeMatrix.at<double>(0,0) = ddata[14];
        handeyeMatrix.at<double>(0,1) = ddata[15];
        handeyeMatrix.at<double>(0,2) = ddata[16];
        handeyeMatrix.at<double>(0,3) = ddata[17];
        handeyeMatrix.at<double>(1,0) = ddata[18];
        handeyeMatrix.at<double>(1,1) = ddata[19];
        handeyeMatrix.at<double>(1,2) = ddata[20];
        handeyeMatrix.at<double>(1,3) = ddata[21];
        handeyeMatrix.at<double>(2,0) = ddata[22];
        handeyeMatrix.at<double>(2,1) = ddata[23];
        handeyeMatrix.at<double>(2,2) = ddata[24];
        handeyeMatrix.at<double>(2,3) = ddata[25];
        handeyeMatrix.at<double>(3,0) = ddata[26];
        handeyeMatrix.at<double>(3,1) = ddata[27];
        handeyeMatrix.at<double>(3,2) = ddata[28];
        handeyeMatrix.at<double>(3,3) = ddata[29];
        cout<<"=================cameraMatrix================"<<endl;
        cout<<cameraMatrix<<endl;
        cout<<"=================distCoeffs=================="<<endl;
        cout<<distCoeffs<<endl;
        cout<<"================handeyeMatrix================"<<endl;
        cout<<handeyeMatrix<<endl;


        //=============================main function================================//
        CalcRtBetweenPics calcrt(&img,&img2,&cameraMatrix,&distCoeffs,&handeyeMatrix);
        cout<<"=================grasp1:=============="<<endl<<grasp1<<endl;
        cv::Mat delta_grasp(4, 4, CV_64FC1);
        calcrt.getHand2ObjMatrix(&delta_grasp);
        grasp2 = grasp1*delta_grasp;
        cout<<"=================grasp2:=============="<<endl<<grasp2<<endl;
        //===========================================================================//


        //=============================servo move====================================//
        geometry_msgs::Pose target_pose1;

        double q0 = sqrt(1 + grasp2.at<double>(0,0) + grasp2.at<double>(1,1) + grasp2.at<double>(2,2))/2.0;
        target_pose1.orientation.x = (grasp2.at<double>(2,1) - grasp2.at<double>(1,2))/(4*q0);
        target_pose1.orientation.y = (grasp2.at<double>(0,2) - grasp2.at<double>(2,0))/(4*q0);
        target_pose1.orientation.z = (grasp2.at<double>(1,0) - grasp2.at<double>(0,1))/(4*q0);
        target_pose1.orientation.w = q0;
        target_pose1.position.x = grasp2.at<double>(0,3);
        target_pose1.position.y = grasp2.at<double>(1,3);
        target_pose1.position.z = grasp2.at<double>(2,3);
        cout<<"======================target_start:====================="<<endl<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<" qw:"<<d<<"qx:"<<a<<" qy:"<<b<<" qz:"<<c<<endl;
        cout<<"======================target_end:======================="<<endl<<"x:"<<target_pose1.position.x<<",y:"<<target_pose1.position.y<<",z:"<<target_pose1.position.z<<",qw:"<<target_pose1.orientation.w
           <<",qx:"<<target_pose1.orientation.x<<",qy:"<<target_pose1.orientation.y<<",qz:"<<target_pose1.orientation.z<<endl;


        robot_state::RobotState start_state_2(*robot_arm.getCurrentState());
        const robot_state::JointModelGroup *joint_model_group_2 = start_state_2.getJointModelGroup(robot_arm.getName());
        bool success_ik;

        for(int i=0;i<100;i++)
        {
            cout<<"num of ik:"<<i<<endl;
            success_ik = start_state_2.setFromIK(joint_model_group_2, target_pose1);
            if(success_ik)
            {break;}
        }
        cout<<"======================find grasp pose:set from ik is success?:======================"<<endl<<success_ik<<endl;
        if(success_ik)
        {
           std::vector<double> joint_values0;
           joint_values0.push_back(arm.joint1); joint_values0.push_back(arm.joint2);
           joint_values0.push_back(arm.joint3); joint_values0.push_back(arm.joint4);
           joint_values0.push_back(arm.joint5); joint_values0.push_back(arm.joint6);
           std::vector<double> joint_values;
           std::vector<double> joint_values2;
           start_state_2.copyJointGroupPositions(joint_model_group_2, joint_values);
           const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
           for(std::size_t i = 0; i < 6; ++i)
           {
               double t;
               ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
               if( abs(joint_values[i]-joint_values0[i]) > abs(joint_values[i]+2*M_PI-joint_values0[i]) )
               {
                   t = joint_values[i]+2*M_PI;
               }
               else if( abs(joint_values[i]-joint_values0[i]) > abs(joint_values[i]-2*M_PI-joint_values0[i]) )
               {
                   t = joint_values[i]-2*M_PI;
               }
               else
               {
                   t = joint_values[i];
               }
               joint_values2.push_back(t);
               cout<<"joint "<<i<<" :"<<t<<endl;
           }

           ArmJoints arm2(joint_values2[0], joint_values2[1], joint_values2[2], joint_values2[3], joint_values2[4], joint_values2[5]);






           double  q_j[6];

           int Step_Num = 20;

           q_j[0] = arm.joint1; q_j[1] = arm.joint2; q_j[2] = arm.joint3;
           q_j[3] = arm.joint4; q_j[4] = arm.joint5; q_j[5] = arm.joint6;


           int start_idx = 0;
           vector<ArmJoints> Whole_joints_of_all_nodes(Step_Num);


           Whole_joints_of_all_nodes[0].joint1 = arm.joint1;
           Whole_joints_of_all_nodes[0].joint2 = arm.joint2;
           Whole_joints_of_all_nodes[0].joint3 = arm.joint3;
           Whole_joints_of_all_nodes[0].joint4 = arm.joint4;
           Whole_joints_of_all_nodes[0].joint5 = arm.joint5;
           Whole_joints_of_all_nodes[0].joint6 = arm.joint6;


           planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
           robot_state::RobotState current_robot_state = current_scene->getCurrentState();
           robot_arm.setStartState(current_robot_state);

           double resol_1=0.6;double resol_2=0.6;double resol_3=0.6;double resol_4=0.6;double resol_5=0.6;double resol_6=0.6;
           bool  found=0;
           double lase_distance_thread=2.0;
           vector<ArmJoints> Whole_joints_of_all_nodes_last(Step_Num+4);
           int NUM_CIR=100;
           for(int i=0;i<NUM_CIR;i++)
           {
               double last_distance;
               bool jump=0;
               found=0;

               bool Is_Colliding=0;
               int k;
               int num_of_retry=5;
               int calc_collision;

               double n_proportion = 3;
               double relax_law = 1.3;

               for( k=start_idx+1;k<Step_Num;k++)
               {
                   calc_collision=0;
                   for(int retry=0;retry<num_of_retry;retry++)
                   {
                       vector<double> forward_joints;
                       vector<double> back_joints;
                           q_j[0]=Whole_joints_of_all_nodes[k-1].joint1+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint1-Whole_joints_of_all_nodes[k-1].joint1)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));
                           q_j[1]=Whole_joints_of_all_nodes[k-1].joint2+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint2-Whole_joints_of_all_nodes[k-1].joint2)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));
                           q_j[2]=Whole_joints_of_all_nodes[k-1].joint3+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint3-Whole_joints_of_all_nodes[k-1].joint3)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));
                           q_j[3]=Whole_joints_of_all_nodes[k-1].joint4+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint4-Whole_joints_of_all_nodes[k-1].joint4)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));
                           q_j[4]=Whole_joints_of_all_nodes[k-1].joint5+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint5-Whole_joints_of_all_nodes[k-1].joint5)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));
                           q_j[5]=Whole_joints_of_all_nodes[k-1].joint6+relax_law*(2*n_proportion/(n_proportion-2))*((arm2.joint6-Whole_joints_of_all_nodes[k-1].joint6)/(Step_Num-start_idx))*(rand01()-(1/n_proportion));

                           if(k==Step_Num-1)
                           {
                               double distance_between_two_config=sqrt(10*(arm2.joint1-Whole_joints_of_all_nodes[k-1].joint1)*(arm2.joint1-Whole_joints_of_all_nodes[k-1].joint1)+
                                                                       10*(arm2.joint2-Whole_joints_of_all_nodes[k-1].joint2)*(arm2.joint2-Whole_joints_of_all_nodes[k-1].joint2)+
                                                                       10*(arm2.joint3-Whole_joints_of_all_nodes[k-1].joint3)*(arm2.joint3-Whole_joints_of_all_nodes[k-1].joint3)+
                                                                          (arm2.joint4-Whole_joints_of_all_nodes[k-1].joint4)*(arm2.joint4-Whole_joints_of_all_nodes[k-1].joint4)+
                                                                          (arm2.joint5-Whole_joints_of_all_nodes[k-1].joint5)*(arm2.joint5-Whole_joints_of_all_nodes[k-1].joint5)+
                                                                          (arm2.joint6-Whole_joints_of_all_nodes[k-1].joint6)*(arm2.joint6-Whole_joints_of_all_nodes[k-1].joint6));
                               last_distance=distance_between_two_config;
                               cout<<"The "<<i+1<<" th Circle has "<<Step_Num<<" steps,and the "<<k+1<<" th step distance between two config:  "<<distance_between_two_config<<endl;
                           }

                               forward_joints.clear();

                               for(int v=0;v<6;v++)
                               {
                                   forward_joints.push_back(q_j[v]);
                               }

                       back_joints.clear();

                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint1);
                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint2);
                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint3);
                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint4);
                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint5);
                       back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint6);

                       string word[6]={"jaco_joint_1","jaco_joint_2","jaco_joint_3","jaco_joint_4","jaco_joint_5","jaco_joint_6"};
                       std::vector<std::string> joint_names(word,word+6);
                       current_robot_state.copyJointGroupPositions( robot_arm.getName(), back_joints);
                       current_robot_state.setVariablePositions( joint_names, forward_joints);
                       Is_Colliding = current_scene->isStateColliding( current_robot_state );
                       if(Is_Colliding)
                       {
                           calc_collision++;
                           cout<<"calc_collision:"<<calc_collision<<endl;
                           if(retry<num_of_retry-1)
                           {continue;}
                       }
                       else
                       {
                           Whole_joints_of_all_nodes[k].joint1=forward_joints[0];
                           Whole_joints_of_all_nodes[k].joint2=forward_joints[1];
                           Whole_joints_of_all_nodes[k].joint3=forward_joints[2];
                           Whole_joints_of_all_nodes[k].joint4=forward_joints[3];
                           Whole_joints_of_all_nodes[k].joint5=forward_joints[4];
                           Whole_joints_of_all_nodes[k].joint6=forward_joints[5];

                           break;
                       }

                   if(calc_collision==num_of_retry)
                   {
                       jump=1;
                       break;
                   }
                   }
                   if(jump==1)
                   {
                       break;
                   }
               }
              if(jump==1)
              {
                  continue;
              }
              else if(last_distance<lase_distance_thread)
              {
                  cout<<"path found!"<<endl;
                  cout<<"jump:"<<jump<<endl;
                  Whole_joints_of_all_nodes[Step_Num-1].joint1=arm2.joint1;
                  Whole_joints_of_all_nodes[Step_Num-1].joint2=arm2.joint2;
                  Whole_joints_of_all_nodes[Step_Num-1].joint3=arm2.joint3;
                  Whole_joints_of_all_nodes[Step_Num-1].joint4=arm2.joint4;
                  Whole_joints_of_all_nodes[Step_Num-1].joint5=arm2.joint5;
                  Whole_joints_of_all_nodes[Step_Num-1].joint6=arm2.joint6;
                  //Whole_joints_of_all_nodes_last.clear();
                  for(int i=0;i<Step_Num;i++)
                  {
                      Whole_joints_of_all_nodes_last[i].joint1 = Whole_joints_of_all_nodes[i].joint1;
                      Whole_joints_of_all_nodes_last[i].joint2 = Whole_joints_of_all_nodes[i].joint2;
                      Whole_joints_of_all_nodes_last[i].joint3 = Whole_joints_of_all_nodes[i].joint3;
                      Whole_joints_of_all_nodes_last[i].joint4 = Whole_joints_of_all_nodes[i].joint4;
                      Whole_joints_of_all_nodes_last[i].joint5 = Whole_joints_of_all_nodes[i].joint5;
                      Whole_joints_of_all_nodes_last[i].joint6 = Whole_joints_of_all_nodes[i].joint6;
                  }
                  break;
              }
           }
            cout << "++++++++++876+++++++++" << Whole_joints_of_all_nodes_last.size() << "+++++++++++++" << endl;

           //****************************grasp parallel insert*******************************//
           KDL::JntArray  q2_;
           KDL::Jacobian  J2_;
           q2_.resize(6);
           J2_.resize(jaco_chain.getNrOfJoints());
           q2_(0) = arm2.joint1; q2_(1) = arm2.joint2; q2_(2) = arm2.joint3;
           q2_(3) = arm2.joint4; q2_(4) = arm2.joint5; q2_(5) = arm2.joint6;
           jnt_to_jac_solver_->JntToJac(q2_,J2_);
           cv::Mat delta_x(6, 1, CV_64FC1);
           cv::Mat delta_q(6, 1, CV_64FC1);
           cv::Mat Jacobi (6, 6, CV_64FC1);
           double delta_l = 0.02;
           delta_x.at<double>(0,0) = -delta_l*grasp2.at<double>(0,2);
           delta_x.at<double>(1,0) = -delta_l*grasp2.at<double>(1,2);
           delta_x.at<double>(2,0) = -delta_l*grasp2.at<double>(2,2);
           delta_x.at<double>(3,0) = 0;
           delta_x.at<double>(4,0) = 0;
           delta_x.at<double>(5,0) = 0;
           for(int i=0;i<6;i++)
           {
               for(int j=0;j<6;j++)
               {
                    Jacobi.at<double>(i,j) = J2_.data(i,j);
                    //cout<<"Jacobi.at<double>(i,j)"<<Jacobi.at<double>(i,j)<<",J2_.data(i,j)"<<J2_.data(i,j)<<endl;
               }
           }
           delta_q = Jacobi.inv()*delta_x;
           ArmJoints arm3(arm2.joint1 + delta_q.at<double>(0,0), arm2.joint2 + delta_q.at<double>(1,0),
                          arm2.joint3 + delta_q.at<double>(2,0), arm2.joint4 + delta_q.at<double>(3,0),
                          arm2.joint5 + delta_q.at<double>(4,0), arm2.joint6 + delta_q.at<double>(5,0));

           KDL::JntArray  q3_;
           KDL::Jacobian  J3_;
           q3_.resize(6);
           J3_.resize(jaco_chain.getNrOfJoints());
           q3_(0) = arm3.joint1; q3_(1) = arm3.joint2; q3_(2) = arm3.joint3;
           q3_(3) = arm3.joint4; q3_(4) = arm3.joint5; q3_(5) = arm3.joint6;
           jnt_to_jac_solver_->JntToJac(q3_,J3_);
           for(int i=0;i<6;i++)
           {
               for(int j=0;j<6;j++)
               {
                    Jacobi.at<double>(i,j) = J3_.data(i,j);
                    //cout<<"Jacobi.at<double>(i,j)"<<Jacobi.at<double>(i,j)<<",J2_.data(i,j)"<<J2_.data(i,j)<<endl;
               }
           }
           delta_q = Jacobi.inv()*delta_x;
           ArmJoints arm4(arm3.joint1 + delta_q.at<double>(0,0), arm3.joint2 + delta_q.at<double>(1,0),
                          arm3.joint3 + delta_q.at<double>(2,0), arm3.joint4 + delta_q.at<double>(3,0),
                          arm3.joint5 + delta_q.at<double>(4,0), arm3.joint6 + delta_q.at<double>(5,0));

           KDL::JntArray  q4_;
           KDL::Jacobian  J4_;
           q4_.resize(6);
           J4_.resize(jaco_chain.getNrOfJoints());
           q4_(0) = arm4.joint1; q4_(1) = arm4.joint2; q4_(2) = arm4.joint3;
           q4_(3) = arm4.joint4; q4_(4) = arm4.joint5; q4_(5) = arm3.joint6;
           jnt_to_jac_solver_->JntToJac(q4_,J4_);
           for(int i=0;i<6;i++)
           {
               for(int j=0;j<6;j++)
               {
                    Jacobi.at<double>(i,j) = J4_.data(i,j);
                    //cout<<"Jacobi.at<double>(i,j)"<<Jacobi.at<double>(i,j)<<",J2_.data(i,j)"<<J2_.data(i,j)<<endl;
               }
           }
           delta_q = Jacobi.inv()*delta_x;
           ArmJoints arm5(arm4.joint1 + delta_q.at<double>(0,0), arm4.joint2 + delta_q.at<double>(1,0),
                          arm4.joint3 + delta_q.at<double>(2,0), arm4.joint4 + delta_q.at<double>(3,0),
                          arm4.joint5 + delta_q.at<double>(4,0), arm4.joint6 + delta_q.at<double>(5,0));

           KDL::JntArray  q5_;
           KDL::Jacobian  J5_;
           q5_.resize(6);
           J5_.resize(jaco_chain.getNrOfJoints());
           q5_(0) = arm5.joint1; q5_(1) = arm5.joint2; q5_(2) = arm5.joint3;
           q5_(3) = arm5.joint4; q5_(4) = arm5.joint5; q5_(5) = arm5.joint6;
           jnt_to_jac_solver_->JntToJac(q5_,J5_);
           for(int i=0;i<6;i++)
           {
               for(int j=0;j<6;j++)
               {
                    Jacobi.at<double>(i,j) = J5_.data(i,j);
                    //cout<<"Jacobi.at<double>(i,j)"<<Jacobi.at<double>(i,j)<<",J2_.data(i,j)"<<J2_.data(i,j)<<endl;
               }
           }
           delta_q = Jacobi.inv()*delta_x;
           ArmJoints arm6(arm5.joint1 + delta_q.at<double>(0,0), arm5.joint2 + delta_q.at<double>(1,0),
                          arm5.joint3 + delta_q.at<double>(2,0), arm5.joint4 + delta_q.at<double>(3,0),
                          arm5.joint5 + delta_q.at<double>(4,0), arm5.joint6 + delta_q.at<double>(5,0));
           //******************************************************************************//


           Whole_joints_of_all_nodes_last[Step_Num] = arm3;

           Whole_joints_of_all_nodes_last[Step_Num+1] = arm4;

           Whole_joints_of_all_nodes_last[Step_Num+2] = arm5;

           Whole_joints_of_all_nodes_last[Step_Num+3] = arm6;

        cout << "++++++++++979+++++++++" << Whole_joints_of_all_nodes_last.size() << "+++++++++++++" << endl;

           for(int time=0;time<5;time++)
          {
               vector<ArmJoints> curve;

    //           curve.assign(Whole_joints_of_all_nodes_last.begin(),Whole_joints_of_all_nodes_last.end());
               for(int i = 0; i < Whole_joints_of_all_nodes_last.size(); i++) {
                   curve.push_back(Whole_joints_of_all_nodes_last[i]);
               }

               for(int i=1;i<Whole_joints_of_all_nodes_last.size()-1;i++)
               {
               Whole_joints_of_all_nodes_last[i].joint1=(1.0/6.0)*curve[i-1].joint1+(2.0/3.0)*curve[i].joint1+(1.0/6.0)*curve[i+1].joint1;
               Whole_joints_of_all_nodes_last[i].joint2=(1.0/6.0)*curve[i-1].joint2+(2.0/3.0)*curve[i].joint2+(1.0/6.0)*curve[i+1].joint2;
               Whole_joints_of_all_nodes_last[i].joint3=(1.0/6.0)*curve[i-1].joint3+(2.0/3.0)*curve[i].joint3+(1.0/6.0)*curve[i+1].joint3;
               Whole_joints_of_all_nodes_last[i].joint4=(1.0/6.0)*curve[i-1].joint4+(2.0/3.0)*curve[i].joint4+(1.0/6.0)*curve[i+1].joint4;
               Whole_joints_of_all_nodes_last[i].joint5=(1.0/6.0)*curve[i-1].joint5+(2.0/3.0)*curve[i].joint5+(1.0/6.0)*curve[i+1].joint5;
               Whole_joints_of_all_nodes_last[i].joint6=(1.0/6.0)*curve[i-1].joint6+(2.0/3.0)*curve[i].joint6+(1.0/6.0)*curve[i+1].joint6;
               }
           }




               //=====================after plan and public======================//
                moveit_msgs::RobotTrajectory trajectory_msg_;
                trajectory_msgs::JointTrajectory trajectory =  trajectory_msg_.joint_trajectory;
                trajectory.points.resize(Whole_joints_of_all_nodes_last.size());
                trajectory.joint_names.push_back("jaco_joint_1");
                trajectory.joint_names.push_back("jaco_joint_2");
                trajectory.joint_names.push_back("jaco_joint_3");
                trajectory.joint_names.push_back("jaco_joint_4");
                trajectory.joint_names.push_back("jaco_joint_5");
                trajectory.joint_names.push_back("jaco_joint_6");

                int num_of_joints=6;

                for(int i=0;i<Whole_joints_of_all_nodes_last.size();i++)
                {
                    trajectory.points[i].positions.resize(num_of_joints);
                    trajectory.points[i].velocities.resize(num_of_joints);
                    trajectory.points[i].accelerations.resize(num_of_joints);
                    trajectory.points[i].positions[0] = Whole_joints_of_all_nodes_last[i].joint1;
                    trajectory.points[i].velocities[0] = 0.001;
                    trajectory.points[i].accelerations[0] = 0.005;
                    trajectory.points[i].positions[1] = Whole_joints_of_all_nodes_last[i].joint2;
                    trajectory.points[i].velocities[1] = 0.001;
                    trajectory.points[i].accelerations[1] = 0.005;
                    trajectory.points[i].positions[2] = Whole_joints_of_all_nodes_last[i].joint3;
                    trajectory.points[i].velocities[2] = 0.001;
                    trajectory.points[i].accelerations[2] = 0.005;
                    trajectory.points[i].positions[3] = Whole_joints_of_all_nodes_last[i].joint4;
                    trajectory.points[i].velocities[3] = 0.001;
                    trajectory.points[i].accelerations[3] = 0.005;
                    trajectory.points[i].positions[4] = Whole_joints_of_all_nodes_last[i].joint5;
                    trajectory.points[i].velocities[4] = 0.001;
                    trajectory.points[i].accelerations[4] = 0.005;
                    trajectory.points[i].positions[5] = Whole_joints_of_all_nodes_last[i].joint6;
                    trajectory.points[i].velocities[5] = 0.001;
                    trajectory.points[i].accelerations[5] = 0.005;
                }
                 cout<<"path size:"<<Whole_joints_of_all_nodes_last.size()<<endl;

                moveit::planning_interface::MoveGroup::Plan mani_plan;
                trajectory_msg_.joint_trajectory=trajectory;
                mani_plan.trajectory_=trajectory_msg_;
                moveit_msgs::DisplayTrajectory display_trajectory;
                display_trajectory.trajectory_start=mani_plan.start_state_;
                display_trajectory.trajectory.push_back(mani_plan.trajectory_);
                display_publisher.publish(display_trajectory);

                ROS_INFO("++++++begin 1052+++++++++");
                bool _return=robot_arm.execute(mani_plan);
                ROS_INFO("Visualizing simultanious plan1 manifold %s",_return?" ":"Failed");



                //=====================grasp plan and public======================//

                finger_client_.waitForServer();
                goal_f.fingers.finger1 = 30;
                goal_f.fingers.finger2 = 30;
                goal_f.fingers.finger3 = 30;
                finger_client_.sendGoal(goal_f);
                bool finish_before_timeout = finger_client_.waitForResult(ros::Duration(5.0));

                //==========================================================================//



        }








        //===========================================================================//



    //==================================================================================//
    ROS_INFO("Finished");
//    ros::spin();
    return 0;
}
