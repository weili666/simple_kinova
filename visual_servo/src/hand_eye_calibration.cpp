#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <Eigen/Geometry>
#include <ctime>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/core/eigen.hpp>
#include <algorithm>


#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
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
using namespace std;
using namespace cv;
int num=0;

Mat skew(Mat_<double> p)
{
    Mat P(3, 3, CV_64FC1);
    P.at<double>(0,0)=0;
    P.at<double>(0,1)=-p.at<double>(2,0);
    P.at<double>(0,2)=p.at<double>(1,0);
    P.at<double>(1,0)=p.at<double>(2,0);
    P.at<double>(1,1)=0;
    P.at<double>(1,2)=-p.at<double>(0,0);
    P.at<double>(2,0)=-p.at<double>(1,0);
    P.at<double>(2,1)=p.at<double>(0,0);
    P.at<double>(2,2)=0;
    return P;
}

Mat cross(const Mat& a,const Mat& b)
{
    Mat c(3, 1, CV_64FC1);
    c.at<double>(0,0) = a.at<double>(1,0)*b.at<double>(2,0)-a.at<double>(2,0)*b.at<double>(1,0);
    c.at<double>(1,0) = a.at<double>(2,0)*b.at<double>(0,0)-a.at<double>(0,0)*b.at<double>(2,0);
    c.at<double>(2,0) = a.at<double>(0,0)*b.at<double>(1,0)-a.at<double>(1,0)*b.at<double>(0,0);
    return c;
}

Mat qmult(const Eigen::Quaterniond& q1,const Eigen::Quaterniond& q2)
{
    Mat prod1(4, 1, CV_64FC1);
    prod1.at<double>(0,0) = q1.x()*q2.w(); prod1.at<double>(1,0) = -q1.x()*q2.z(); prod1.at<double>(2,0) = q1.x()*q2.y(); prod1.at<double>(3,0) = -q1.x()*q2.x();
    Mat prod2(4, 1, CV_64FC1);
    prod2.at<double>(0,0) = q1.y()*q2.z(); prod2.at<double>(1,0) = q1.y()*q2.w(); prod2.at<double>(2,0) = -q1.y()*q2.x(); prod2.at<double>(3,0) = -q1.y()*q2.y();
    Mat prod3(4, 1, CV_64FC1);
    prod3.at<double>(0,0) = -q1.z()*q2.y(); prod3.at<double>(1,0) = q1.z()*q2.x(); prod3.at<double>(2,0) = q1.z()*q2.w(); prod3.at<double>(3,0) = -q1.z()*q2.z();
    Mat prod4(4, 1, CV_64FC1);
    prod4.at<double>(0,0) = q1.w()*q2.x(); prod4.at<double>(1,0) = q1.w()*q2.y(); prod4.at<double>(2,0) = q1.w()*q2.z(); prod4.at<double>(3,0) = q1.w()*q2.w();
    Mat q_out(4, 1, CV_64FC1);
    q_out = prod1 + prod2 + prod3 + prod4;
    return q_out;
}

class Dual_Quaternion
{
public:
    Dual_Quaternion(Eigen::Quaterniond& q_,Eigen::Quaterniond& qprime_):q(q_),qprime(qprime_){}
    Dual_Quaternion(){}
    Eigen::Quaterniond q;
    Eigen::Quaterniond qprime;
};

class HandEyeCalibration
{
public:
    HandEyeCalibration(int num_cols,int num_rows,double dist,const vector<geometry_msgs::Pose>& pose_vec):target_poses(pose_vec)
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
        char buff[100];

        int num_of_pic=target_poses.size();
        cout<<"num of picture:"<<num_of_pic<<endl;
        //===========================DETECT CORNER============================//
        for(int i=1;i<=num_of_pic;i++)
        {
            sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/saved_image/saved_%d.png",i);
            cout<<buff<<endl;

            img = cv::imread(buff);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);

            bool find=cv::findChessboardCorners(calpic,patternSize,corners);
            cv::cornerSubPix(calpic, corners, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

            cornersVect.push_back(corners);
            worldPointsVect.push_back(worldPoints);

            cv::drawChessboardCorners(calpic,patternSize,corners,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/simple_kinova/src/visual_servo/saved_image/draw_chessboard_coners_%d.png",i);
            cv::imwrite(buff2, calpic);
        }
        //===================================================================//

        cv::calibrateCamera(worldPointsVect,cornersVect,calpic.size(),cameraMatrix,distCoeffs,rvecs,tvecs);

        //============================RE-PROJECT=============================//
        for(int i=1;i<=num_of_pic;i++)
        {
            sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/saved_image/saved_%d.png",i);
            cout<<buff<<endl;
            img = cv::imread(buff);
            cv::cvtColor(img, calpic, CV_BGR2GRAY);
            std::vector<cv::Point2f> projectedPoints;
            cv::projectPoints(worldPointsVect[i-1],rvecs[i-1],tvecs[i-1],cameraMatrix,distCoeffs,projectedPoints);
            cornersVectProj.push_back(projectedPoints);

            cv::drawChessboardCorners(calpic,patternSize,projectedPoints,1);
            char buff2[100];
            sprintf(buff2, "/home/weili/simple_kinova/src/visual_servo/saved_image/draw_chessboard_project_coners_%d.png",i);
            cv::imwrite(buff2, calpic);
            cv::undistort(img,calpic,cameraMatrix,distCoeffs);
            char buff3[100];
            sprintf(buff3, "/home/weili/simple_kinova/src/visual_servo/saved_image/undistort_%d.png",i);
            cv::imwrite(buff3, calpic);

        }
        //===================================================================//

        //===================================================================//

        cout<<"<<<<<<<<<<<<<---------calpic_size-------->>>>>>>>>>>>>>>"<<endl;
        cout<<calpic.size()<<endl;
        cout<<"size of connersVect:"<<cornersVect.size()<<endl;
        cout<<"<<<<<<<<<<<<<---------cameraMatrix-------->>>>>>>>>>>>>>>"<<endl;
        cout<<cameraMatrix<<endl;
        cout<<"<<<<<<<<<<<<<-----------distCoeffs-------->>>>>>>>>>>>>>>"<<endl;
        cout<<distCoeffs<<endl;


        for(int k=1;k<=num_of_pic;k++)
        {
            cv::Mat rotMatrixi;
            cv::Mat Grasp_matrix_rt(4, 4, CV_64FC1);
            cv::Mat Camera_matrix_rt(4, 4, CV_64FC1);
            cv::Rodrigues(rvecs[k-1],rotMatrixi,JacoMatrix);
            rotMatrix = rotMatrixi.inv();
            cv::Mat tveci = tvecs[k-1];
            cv::Mat tvec = rotMatrix*(-tveci);
            cv::Mat rotMat(3, 3, CV_64FC1);
            cv::Mat tranMat(3, 1, CV_64FC1);
            //==========================Camera==========================//
            for (int i=0;i<rotMatrix.rows;i++)
            {
                for (int j=0;j<rotMatrix.cols;j++)
                {
                    rotMat.at<double>(i,j)=rotMatrix.at<double>(i,j);
                }
            }
            for (int i=0;i<3;i++)
            {
                tranMat.at<double>(i,0)= tvec.at<double>(i,0)/1000.0;
            }
            Camera_Matrix_r.push_back(rotMat);
            Camera_Matrix_t.push_back(tranMat);
            rotMat.copyTo(Camera_matrix_rt(Rect(0, 0, 3, 3)));
            tranMat.copyTo(Camera_matrix_rt(Rect(3, 0, 1, 3)));
            Camera_matrix_rt.at<double>(3, 0) = 0.0;
            Camera_matrix_rt.at<double>(3, 1) = 0.0;
            Camera_matrix_rt.at<double>(3, 2) = 0.0;
            Camera_matrix_rt.at<double>(3, 3) = 1.0;
            Camera_Matrix_rt.push_back(Camera_matrix_rt);
            //==========================================================//

            //==========================Grasp==========================//
            geometry_msgs::Pose pose=target_poses[k-1];
            Eigen::Quaterniond q (pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
            Eigen::Matrix3d G_Mat(q);
            cv::Mat grasp_mat(3, 3, CV_64FC1);
            for(int i=0;i<grasp_mat.rows;i++)
            {
                for(int j=0;j<grasp_mat.cols;j++)
                {
                    grasp_mat.at<double>(i,j)=G_Mat(i,j);
                }
            }
            cv::Mat grasp_mat_t(3, 1, CV_64FC1);
            grasp_mat_t.at<double>(0,0)=pose.position.x;
            grasp_mat_t.at<double>(1,0)=pose.position.y;
            grasp_mat_t.at<double>(2,0)=pose.position.z;
            Grasp_Matrix_r.push_back(grasp_mat);
            Grasp_Matrix_t.push_back(grasp_mat_t);

            grasp_mat.copyTo(Grasp_matrix_rt(Rect(0, 0, 3, 3)));
            grasp_mat_t.copyTo(Grasp_matrix_rt(Rect(3, 0, 1, 3)));
            Grasp_matrix_rt.at<double>(3, 0) = 0.0;
            Grasp_matrix_rt.at<double>(3, 1) = 0.0;
            Grasp_matrix_rt.at<double>(3, 2) = 0.0;
            Grasp_matrix_rt.at<double>(3, 3) = 1.0;
            Grasp_Matrix_rt.push_back(Grasp_matrix_rt);
            //==========================================================//
        }

        for(int i=0;i<num_of_pic;i++)
        {
            cout<<"======================Camera_Matrix_rt "<<i+1<<" ===================="<<endl;
            cout<<Camera_Matrix_rt[i]<<endl;

            cout<<"======================Grasp_Matrix_rt "<<i+1<<" ===================="<<endl;
            cout<<Grasp_Matrix_rt[i]<<endl;

        }


        int k=1;
        for(int i=0;i<Grasp_Matrix_r.size()-k;i++)
        {
            cv::Mat grasp_mat_rt(4, 4, CV_64FC1);
            cv::Mat camera_mat_rt(4, 4, CV_64FC1);

            grasp_mat_rt = (Grasp_Matrix_rt[i].inv())*Grasp_Matrix_rt[i+k];
            camera_mat_rt = (Camera_Matrix_rt[i].inv())*Camera_Matrix_rt[i+k];
            double DIST_GRASP = sqrt((Grasp_Matrix_t[i+k].at<double>(0,0)-Grasp_Matrix_t[i].at<double>(0,0))*(Grasp_Matrix_t[i+k].at<double>(0,0)-Grasp_Matrix_t[i].at<double>(0,0))
                    +(Grasp_Matrix_t[i+k].at<double>(1,0)-Grasp_Matrix_t[i].at<double>(1,0))*(Grasp_Matrix_t[i+k].at<double>(1,0)-Grasp_Matrix_t[i].at<double>(1,0))
                    +(Grasp_Matrix_t[i+k].at<double>(2,0)-Grasp_Matrix_t[i].at<double>(2,0))*(Grasp_Matrix_t[i+k].at<double>(2,0)-Grasp_Matrix_t[i].at<double>(2,0)));

            double DIST_CAMERA = sqrt((Camera_Matrix_t[i+k].at<double>(0,0)-Camera_Matrix_t[i].at<double>(0,0))*(Camera_Matrix_t[i+k].at<double>(0,0)-Camera_Matrix_t[i].at<double>(0,0))
                    +(Camera_Matrix_t[i+k].at<double>(1,0)-Camera_Matrix_t[i].at<double>(1,0))*(Camera_Matrix_t[i+k].at<double>(1,0)-Camera_Matrix_t[i].at<double>(1,0))
                    +(Camera_Matrix_t[i+k].at<double>(2,0)-Camera_Matrix_t[i].at<double>(2,0))*(Camera_Matrix_t[i+k].at<double>(2,0)-Camera_Matrix_t[i].at<double>(2,0)));

            cout<<"DIST_GRASP:"<<DIST_GRASP<<" ,DIST_CAMERA:"<<DIST_CAMERA<<endl;

            double DIST_GRASP_2 = sqrt(grasp_mat_rt.at<double>(0,3)*grasp_mat_rt.at<double>(0,3)+grasp_mat_rt.at<double>(1,3)*grasp_mat_rt.at<double>(1,3)+grasp_mat_rt.at<double>(2,3)*grasp_mat_rt.at<double>(2,3));

            double DIST_CAMERA_2 = sqrt(camera_mat_rt.at<double>(0,3)*camera_mat_rt.at<double>(0,3)+camera_mat_rt.at<double>(1,3)*camera_mat_rt.at<double>(1,3)+camera_mat_rt.at<double>(2,3)*camera_mat_rt.at<double>(2,3));

            cout<<"DIST_GRASP_2:"<<DIST_GRASP_2<<" ,DIST_CAMERA_2:"<<DIST_CAMERA_2<<endl;

            cout<<"------------------------GraspMatrixRT "<<i+1<<" ------------------"<<endl;
            cout<<grasp_mat_rt<<endl;

            cout<<"------------------------CameraMatrixRT "<<i+1<<" ------------------"<<endl;
            cout<<camera_mat_rt<<endl;

            Grasp_Rotate_Transf_Matrix.push_back(grasp_mat_rt);
            Camera_Rotate_Transf_Matrix.push_back(camera_mat_rt);

        }

        cout<<"begin hand eye calibration!!"<<endl;
        Hcg = Tsai_HandEye( Grasp_Rotate_Transf_Matrix,Camera_Rotate_Transf_Matrix);
        cout<<"------------------------Tsai_HandEye Hand-Camera-Matrix 1------------------"<<endl;
        cout<<Hcg<<endl;

        Hcg = Tsai_HandEye( Camera_Rotate_Transf_Matrix,Grasp_Rotate_Transf_Matrix);
        cout<<"------------------------Tsai_HandEye Hand-Camera-Matrix 2------------------"<<endl;
        cout<<Hcg<<endl;

        Hcg = hand_eye_dual_quaternion(Grasp_Matrix_rt,Camera_Matrix_rt);
        cout<<"------------------------hand_eye_dual_quaternion Hand-Camera-Matrix------------------"<<endl;
        cout<<Hcg<<endl;

        Hcg = Navy_HandEye( Camera_Rotate_Transf_Matrix,Grasp_Rotate_Transf_Matrix);
        cout<<"------------------------Navy_HandEye Hand-Camera-Matrix 1------------------"<<endl;
        cout<<Hcg<<endl;

        Hcg = Navy_HandEye( Grasp_Rotate_Transf_Matrix,Camera_Rotate_Transf_Matrix);
        cout<<"------------------------Navy_HandEye Hand-Camera-Matrix 2------------------"<<endl;
        cout<<Hcg<<endl;

        //Hcg = Tensor_HandEye( Camera_Rotate_Transf_Matrix,Grasp_Rotate_Transf_Matrix );


        ofstream file;
        file.open("/home/weili/simple_kinova/src/visual_servo/saved_image/extern_matrix.txt",ios::app|ios::out);
        file<<cameraMatrix<<endl;
        file<<distCoeffs<<endl;
        file<<Hcg<<endl;


    }

    void getDualQuaternion(const Mat& R,const Mat& t,Eigen::Quaterniond& q,Eigen::Quaterniond& qprime);
    Mat Tsai_HandEye(const vector<Mat>& Hgij, const vector<Mat>& Hcij);
    Mat hand_eye_dual_quaternion(const vector<Mat>& Hmarker2world, const vector<Mat>& Hgrid2cam);
    Mat Navy_HandEye(const vector<Mat>& Hgij, const vector<Mat>& Hcij);
    Mat Tensor_HandEye(const vector<Mat>& Hgij,const vector<Mat>& Hcij);
    int delta(int x,int y);

 private:
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
    cv::Mat Hand_Camera_Matrix;
    cv::Mat Camera_Ext;
    cv::Mat Hcg;
    std::vector<cv::Mat> Grasp_Rotate_Transf_Matrix;
    std::vector<cv::Mat> Grasp_Rotate_Matrix;
    std::vector<cv::Mat> Grasp_Transf_Matrix;
    std::vector<cv::Mat> Grasp_Matrix_r;
    std::vector<cv::Mat> Grasp_Matrix_t;

    std::vector<cv::Mat> Camera_Rotate_Transf_Matrix;
    std::vector<cv::Mat> Camera_Rotate_Matrix;
    std::vector<cv::Mat> Camera_Transf_Matrix;
    std::vector<cv::Mat> Camera_Matrix_r;
    std::vector<cv::Mat> Camera_Matrix_t;

    std::vector<cv::Mat> Grasp_Matrix_rt;
    std::vector<cv::Mat> Camera_Matrix_rt;

    std::vector<cv::Mat> rvecs,tvecs;
    vector<geometry_msgs::Pose> target_poses;
};

Mat HandEyeCalibration::hand_eye_dual_quaternion(const vector<Mat>& Hmarker2world, const vector<Mat>& Hgrid2cam)
{
    int n = Hgrid2cam.size();
//    cout<<"the number of matrixs input:"<<n<<endl;
    vector<Dual_Quaternion> Qa,Qb;
    for(int i=0;i<n-1;i++)
    {
        Dual_Quaternion Qai,Qbi;
        Eigen::Quaterniond q,qprime;
        Mat A = (Hmarker2world[i].inv())*Hmarker2world[i+1];
        Mat B = Hgrid2cam[i+1]*(Hgrid2cam[i].inv());
        Mat Ar(3, 3, CV_64FC1);
        Mat At(3, 1, CV_64FC1);
        Mat Br(3, 3, CV_64FC1);
        Mat Bt(3, 1, CV_64FC1);
        A(Rect(0, 0, 3, 3)).copyTo(Ar);
        A(Rect(3, 0, 1, 3)).copyTo(At);
        B(Rect(0, 0, 3, 3)).copyTo(Br);
        B(Rect(3, 0, 1, 3)).copyTo(Bt);
        getDualQuaternion(Ar,At,q,qprime);
        Qai.q = q;
        Qai.qprime = qprime;
        getDualQuaternion(Br,Bt,q,qprime);
        Qbi.q = q;
        Qbi.qprime = qprime;
        Qa.push_back(Qai);
        Qb.push_back(Qbi);
    }

    Mat S(6, 8, CV_64FC1);
    Mat S_trans(8, 6, CV_64FC1);
    Mat S_11(3, 1, CV_64FC1);
    Mat S_12(3, 3, CV_64FC1);
    Mat S_13(3, 1, CV_64FC1,Scalar::all(0));
    Mat S_14(3, 3, CV_64FC1,Scalar::all(0));
    Mat S_21(3, 1, CV_64FC1);
    Mat S_22(3, 3, CV_64FC1);
    Mat S_23(3, 1, CV_64FC1);
    Mat S_24(3, 3, CV_64FC1);
    Mat tempt(8, 6*(n-1), CV_64FC1);
    for(int i=0;i<n-1;i++)
    {
        Mat M0(3,1,CV_32F);
        M0.at<double>(0,0) = Qa[i].q.x()+Qb[i].q.x(); M0.at<double>(1,0) = Qa[i].q.y()+Qb[i].q.y(); M0.at<double>(2,0) = Qa[i].q.z()+Qb[i].q.z();
        S_11.at<double>(0,0) = Qa[i].q.x()-Qb[i].q.x(); S_11.at<double>(1,0) = Qa[i].q.y()-Qb[i].q.y(); S_11.at<double>(2,0) = Qa[i].q.z()-Qb[i].q.z();
        S_12 = skew(M0);

        Mat M1(3,1,CV_32F);
        M1.at<double>(0,0) = Qa[i].qprime.x()+Qb[i].qprime.x(); M1.at<double>(1,0) = Qa[i].qprime.y()+Qb[i].qprime.y(); M1.at<double>(2,0) = Qa[i].qprime.z()+Qb[i].qprime.z();
        S_21.at<double>(0,0) = Qa[i].qprime.x()-Qb[i].qprime.x(); S_21.at<double>(1,0) = Qa[i].qprime.y()-Qb[i].qprime.y(); S_21.at<double>(2,0) = Qa[i].qprime.z()-Qb[i].qprime.z();
        S_22 = skew(M1);
        S_23.at<double>(0,0) = Qa[i].q.x()-Qb[i].q.x(); S_23.at<double>(1,0) = Qa[i].q.y()-Qb[i].q.y(); S_23.at<double>(2,0) = Qa[i].q.z()-Qb[i].q.z();
        S_24 = skew(M0);

        S_11.copyTo(S(Rect(0, 0, 1, 3)));
        S_12.copyTo(S(Rect(1, 0, 3, 3)));
        S_13.copyTo(S(Rect(4, 0, 1, 3)));
        S_14.copyTo(S(Rect(5, 0, 3, 3)));

        S_21.copyTo(S(Rect(0, 3, 1, 3)));
        S_22.copyTo(S(Rect(1, 3, 3, 3)));
        S_23.copyTo(S(Rect(4, 3, 1, 3)));
        S_24.copyTo(S(Rect(5, 3, 3, 3)));

        transpose(S,S_trans);
        S_trans.copyTo(tempt(Rect(6*i ,0 , 6, 8)));

    }
    Mat U, W, V;
    Mat tempt_trans(6*(n-1), 8, CV_64FC1);
    transpose(tempt,tempt_trans);

    cv::SVD::compute(tempt_trans, W, U, V);

    Mat v7_trans(1, 8, CV_64FC1);
    Mat v8_trans(1, 8, CV_64FC1);
    Mat v7(8, 1, CV_64FC1);
    Mat v8(8, 1, CV_64FC1);
    Mat u1(4, 1, CV_64FC1);
    Mat u1_trans(1, 4, CV_64FC1);
    Mat v1(4, 1, CV_64FC1);
    Mat u2(4, 1, CV_64FC1);
    Mat u2_trans(1, 4, CV_64FC1);
    Mat v2(4, 1, CV_64FC1);
    v7_trans = V.row(6);
    v8_trans = V.row(7);
    transpose(v7_trans,v7);
    transpose(v8_trans,v8);

    v7(Rect(0, 0, 1, 4)).copyTo(u1);
    v7(Rect(0, 4, 1, 4)).copyTo(v1);
    v8(Rect(0, 0, 1, 4)).copyTo(u2);
    v8(Rect(0, 4, 1, 4)).copyTo(v2);
    transpose(u1,u1_trans);
    transpose(u2,u2_trans);
    double a,b,c;
    Mat aa,bb,cc;
    aa = u1_trans*v1;
    bb = u1_trans*v2+u2_trans*v1;
    cc = u2_trans*v2;
    a = aa.at<double>(0,0);
    b = bb.at<double>(0,0);
    c = cc.at<double>(0,0);
    double solve,solve1,solve2;
    double val,val1,val2;
    Mat vval1,vval2;
    solve1 = (-b+sqrt(b*b-4*a*c))/(2*a);
    solve2 = (-b-sqrt(b*b-4*a*c))/(2*a);
    vval1 = solve1*solve1*u1_trans*u1+2*solve1*u1_trans*u2+u2_trans*u2;
    vval2 = solve2*solve1*u1_trans*u1+2*solve2*u1_trans*u2+u2_trans*u2;
    val1 = vval1.at<double>(0,0);
    val2 = vval2.at<double>(0,0);
    if(val1>val2)
    {
        solve = solve1;
        val = val1;
    }
    else
    {
        solve = solve2;
        val = val2;
    }
    double lambda1,lambda2;
    lambda2 = sqrt(1/val);
    lambda1 = solve*lambda2;
    Mat qfinal(8, 1, CV_64FC1);
    qfinal = lambda1*v7+lambda2*v8;
    Eigen::Quaterniond q(qfinal.at<double>(0,0),qfinal.at<double>(1,0),qfinal.at<double>(2,0),qfinal.at<double>(3,0));
    Eigen::Quaterniond qprime(qfinal.at<double>(4,0),qfinal.at<double>(5,0),qfinal.at<double>(6,0),qfinal.at<double>(7,0));
    Eigen::Quaterniond q_conj(q.w(),-q.x(),-q.y(),-q.z());
    q.normalize();


    double q1q1 = q.x()*q.x();
    double q1q2 = q.x()*q.y();
    double q1q3 = q.x()*q.z();
    double q1q4 = q.x()*q.w();
    double q2q2 = q.y()*q.y();
    double q2q3 = q.y()*q.z();
    double q2q4 = q.y()*q.w();
    double q3q3 = q.z()*q.z();
    double q3q4 = q.z()*q.w();
    double q4q4 = q.w()*q.w();

    Mat R(3, 3, CV_64FC1);
    Mat t_all(4, 1, CV_64FC1);
    Mat t(3, 1, CV_64FC1);
    Mat t_(3, 1, CV_64FC1);
    R.at<double>(0,0) = q1q1 - q2q2 - q3q3 + q4q4;
    R.at<double>(0,1) = 2*(q1q2 + q3q4);
    R.at<double>(0,2) = 2*(q1q3 - q2q4);
    R.at<double>(1,0) = 2*(q1q2 - q3q4);
    R.at<double>(1,1) = -q1q1 + q2q2 - q3q3 + q4q4;
    R.at<double>(1,2) = 2*(q2q3 + q1q4);
    R.at<double>(2,0) = 2*(q1q3 + q2q4);
    R.at<double>(2,1) = 2*(q2q3 - q1q4);
    R.at<double>(2,2) = -q1q1 - q2q2 + q3q3 + q4q4;
    t_all = 2*qmult(qprime,q_conj);
    t_all(Rect(0, 0, 1, 3)).copyTo(t);
    Mat H(4, 4, CV_64FC1);
    R.copyTo(H(Rect(0,0,3,3)));
    t_ = -R*t;
    t_.copyTo(H(Rect(3,0,1,3)));
    H.at<double>(3,0) = 0; H.at<double>(3,1) = 0; H.at<double>(3,2) = 0; H.at<double>(3,3) = 1;
    return H;

}

Mat HandEyeCalibration::Tsai_HandEye(const vector<Mat>& Hgij,const vector<Mat>& Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Hcg_(4, 4, CV_64FC1);
    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);

    Mat rgij(3, 1, CV_64FC1);
    Mat rcij(3, 1, CV_64FC1);

    double theta_gij;
    double theta_cij;

    Mat rngij(3, 1, CV_64FC1);
    Mat rncij(3, 1, CV_64FC1);

    Mat Pgij(3, 1, CV_64FC1);
    Mat Pcij(3, 1, CV_64FC1);

    Mat tempA(3, 3, CV_64FC1);
    Mat tempb(3, 1, CV_64FC1);

    Mat A;
    Mat b;
    Mat pinA;

    Mat Pcg_prime(3, 1, CV_64FC1);
    Mat Pcg(3, 1, CV_64FC1);
    Mat PcgTrs(1, 3, CV_64FC1);

    Mat Rcg(3, 3, CV_64FC1);

    Mat eyeM = Mat::eye(3, 3, CV_64FC1);

    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat tempAA(3, 3, CV_64FC1);
    Mat tempbb(3, 1, CV_64FC1);

    Mat AA;
    Mat bb;
    Mat pinAA;

    Mat Tcg(3, 1, CV_64FC1);

    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        Rodrigues(Rgij, rgij);
        Rodrigues(Rcij, rcij);

        theta_gij = norm(rgij);
        theta_cij = norm(rcij);

        rngij = rgij / theta_gij;
        rncij = rcij / theta_cij;

        Pgij = 2 * sin(theta_gij / 2)*rngij;
        Pcij = 2 * sin(theta_cij / 2)*rncij;

        tempA = skew(Pgij + Pcij);
        tempb = Pcij - Pgij;

        A.push_back(tempA);
        b.push_back(tempb);

    }


    //Compute rotation

    invert(A, pinA, DECOMP_SVD);
    Pcg_prime = pinA * b;
    Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
    PcgTrs = Pcg.t();
    Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM + 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));



    //Computer Translation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);


        tempAA = Rgij - eyeM;
        tempbb = Rcg * Tcij - Tgij;

        AA.push_back(tempAA);
        bb.push_back(tempbb);
    }

    invert(AA, pinAA, DECOMP_SVD);
    Tcg = pinAA * bb;

    Rcg.copyTo(Hcg_(Rect(0, 0, 3, 3)));
    Tcg.copyTo(Hcg_(Rect(3, 0, 1, 3)));
    Hcg_.at<double>(3, 0) = 0.0;
    Hcg_.at<double>(3, 1) = 0.0;
    Hcg_.at<double>(3, 2) = 0.0;
    Hcg_.at<double>(3, 3) = 1.0;
    return Hcg_;
}

Mat HandEyeCalibration::Navy_HandEye(const vector<Mat>& Hgij,const vector<Mat>& Hcij)
{
    CV_Assert(Hgij.size() == Hcij.size());
    int nStatus = Hgij.size();

    Mat Hcg(4, 4, CV_64FC1);
    Mat Rgij(3, 3, CV_64FC1);
    Mat Rcij(3, 3, CV_64FC1);

    Mat alpha1(3, 1, CV_64FC1);
    Mat beta1(3, 1, CV_64FC1);
    Mat alpha2(3, 1, CV_64FC1);
    Mat beta2(3, 1, CV_64FC1);
    Mat A(3, 3, CV_64FC1);
    Mat B(3, 3, CV_64FC1);

    Mat alpha(3, 1, CV_64FC1);
    Mat beta(3, 1, CV_64FC1);
    Mat M(3, 3, CV_64FC1, Scalar(0));

    Mat MtM(3, 3, CV_64FC1);
    Mat veMtM(3, 3, CV_64FC1);
    Mat vaMtM(3, 1, CV_64FC1);
    Mat pvaM(3, 3, CV_64FC1, Scalar(0));

    Mat Rx(3, 3, CV_64FC1);

    Mat Tgij(3, 1, CV_64FC1);
    Mat Tcij(3, 1, CV_64FC1);

    Mat eyeM = Mat::eye(3, 3, CV_64FC1);

    Mat tempCC(3, 3, CV_64FC1);
    Mat tempdd(3, 1, CV_64FC1);

    Mat C;
    Mat d;
    Mat Tx(3, 1, CV_64FC1);

    //Compute rotation

    if (Hgij.size() == 2) // Two (Ai,Bi) pairs
    {
        Rodrigues(Hgij[0](Rect(0, 0, 3, 3)), alpha1);
        Rodrigues(Hgij[1](Rect(0, 0, 3, 3)), alpha2);
        Rodrigues(Hcij[0](Rect(0, 0, 3, 3)), beta1);
        Rodrigues(Hcij[1](Rect(0, 0, 3, 3)), beta2);

        alpha1.copyTo(A.col(0));
        alpha2.copyTo(A.col(1));
        (alpha1.cross(alpha2)).copyTo(A.col(2));

        beta1.copyTo(B.col(0));
        beta2.copyTo(B.col(1));
        (beta1.cross(beta2)).copyTo(B.col(2));

        Rx = A*B.inv();

    }
    else // More than two (Ai,Bi) pairs
    {
        for (int i = 0; i < nStatus; i++)
        {
            Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
            Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);

            Rodrigues(Rgij, alpha);
            Rodrigues(Rcij, beta);

            M = M + beta*alpha.t();
        }

        MtM = M.t()*M;
        cv::eigen(MtM, vaMtM, veMtM);

        pvaM.at<double>(0, 0) = 1 / sqrt(vaMtM.at<double>(0, 0));
        pvaM.at<double>(1, 1) = 1 / sqrt(vaMtM.at<double>(1, 0));
        pvaM.at<double>(2, 2) = 1 / sqrt(vaMtM.at<double>(2, 0));

        Rx = veMtM.inv()*pvaM*veMtM*M.t();
    }

   //Computer Translation
    for (int i = 0; i < nStatus; i++)
    {
        Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
        Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
        Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
        Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);

        tempCC = eyeM - Rgij;
        tempdd = Tgij - Rx * Tcij;

        C.push_back(tempCC);
        d.push_back(tempdd);
    }

    Tx = (C.t()*C).inv()*(C.t()*d);

    Rx.copyTo(Hcg(Rect(0, 0, 3, 3)));
    Tx.copyTo(Hcg(Rect(3, 0, 1, 3)));
    Hcg.at<double>(3, 0) = 0.0;
    Hcg.at<double>(3, 1) = 0.0;
    Hcg.at<double>(3, 2) = 0.0;
    Hcg.at<double>(3, 3) = 1.0;

    return Hcg;
}

Mat HandEyeCalibration::Tensor_HandEye(const vector<Mat>& Hgij,const vector<Mat>& Hcij)
{
    Mat A(9, 9, CV_64FC1);
    Mat Hg(3, 3, CV_64FC1);
    Mat Hc(3, 3, CV_64FC1);
    Hg = Hgij[0];
    Hc = Hcij[0];
    for(int i=0;i<9;i++)
    {
        for(int j=0;j<9;j++)
        {
            A.at<double>(i,j) = delta(i,j) - Hg.at<double>(i/3,j/3)*Hc.at<double>(i%3,j%3);
        }
    }
    cv::SVD v_odo(A);
    Mat w,u,vt;
    v_odo.compute(A,w,u,vt);
    cout<<"w:"<<endl<<w<<endl<<"u:"<<endl<<u<<endl<<"vt:"<<endl<<vt<<endl;
}

int HandEyeCalibration::delta(int x,int y)
{
    int result;
    if(x==y)
    {
        result = 1;
    }
    else
    {
        result = 0;
    }
    return result;
}

void HandEyeCalibration::getDualQuaternion(const Mat& R,const Mat& t,Eigen::Quaterniond& q,Eigen::Quaterniond& qprime)
{
//    cout<<"R:"<<R<<endl;
//    cout<<"t:"<<t<<endl;
    Mat r(3, 1, CV_64FC1);
    Mat l(3, 1, CV_64FC1);
    Mat q_trans(1, 3, CV_64FC1);
    Mat qp_im(3, 1, CV_64FC1);
    Mat c(3, 1, CV_64FC1);
    Mat m(3, 1, CV_64FC1);
    Rodrigues(R, r);
    double theta = norm(r);
//    cout<<"theta:"<<theta<<endl;
    l = r/norm(theta);
//    cout<<"l:"<<l<<endl;

    double d = l.at<double>(0,0)*t.at<double>(0,0)+l.at<double>(1,0)*t.at<double>(1,0)+l.at<double>(2,0)*t.at<double>(2,0);
    c = 0.5*(t-d*l)+cos(theta/2)*cross(l,t)/sin(theta/2);
    m = cross(c,l);
    Eigen::Quaterniond q_(cos(theta/2),sin(theta/2)*l.at<double>(0,0),sin(theta/2)*l.at<double>(1,0),sin(theta/2)*l.at<double>(2,0));
//    cout<<"q:"<<q_.w()<<" ,"<<q_.x()<<" ,"<<q_.y()<<" ,"<<q_.z()<<endl;
    transpose(sin(theta/2)*l,q_trans);
    qp_im = 0.5*(cos(theta/2)*t+cross(t,sin(theta/2)*l));
    Eigen::Quaterniond qprime_(-0.5*q_trans.at<double>(0,0)*t.at<double>(0,0)-0.5*q_trans.at<double>(0,1)*t.at<double>(1,0)-0.5*q_trans.at<double>(0,2)*t.at<double>(2,0),qp_im.at<double>(0,0),qp_im.at<double>(1,0),qp_im.at<double>(2,0));
//    cout<<"qprime:"<<qprime_.w()<<" ,"<<qprime_.x()<<" ,"<<qprime_.y()<<" ,"<<qprime_.z()<<endl;
    q = q_;
    qprime = qprime_;
}


int main(int argc, char** argv)
{
    int num_cols = 12;
    int num_rows = 12;
    double dist = 20.286;
//    int num_cols = 8;
//    int num_rows = 6;
//    double dist = 9;

    ifstream ifile;
    ifile.open("/home/weili/simple_kinova/src/visual_servo/saved_image/grasp_pa.txt");
    char buffer[256];

    int num=0;
    int num_of_pic = 32;
    int num_of_pic2 = num_of_pic;
    vector< vector<char> > data_;
    vector<char> data_iter;
    if(!ifile.is_open())
    {cout << "Error opening file"; exit (1);}
    while(num_of_pic)
    {
        data_iter.clear();
        ifile.getline(buffer,100);
        for(int i=0;i<256;i++)
        {
            if(buffer[i]==' ')
            {
                for(int i=0;i<data_iter.size();i++)
                {cout<<data_iter[i];}
                cout<<endl;
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                continue;
            }
            else if(buffer[i]==';')
            {
                for(int i=0;i<data_iter.size();i++)
                {cout<<data_iter[i];}
                cout<<endl;
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
        num_of_pic--;
    }


    int num_data = 7*num_of_pic2;
cout<<"hello world:"<<num_data<<endl;
    double ddata[num_data];
cout<<"hello world2"<<endl;
    for(int i=0;i<num_data;i++)
    {cout<<"hello world3"<<endl;
        string data;

        for(int j=0;j<data_[i].size();j++)
        {
            data+=data_[i][j];
        }

        ddata[i]=atof(data.c_str());

    }

    vector<geometry_msgs::Pose> target_poses;

    for(int i=0;i<num_of_pic2;i++)
    {
        geometry_msgs::Pose targ_pose;
        targ_pose.position.x = ddata[i*7+0];
        targ_pose.position.y = ddata[i*7+1];
        targ_pose.position.z = ddata[i*7+2];
        targ_pose.orientation.x = ddata[i*7+3];
        targ_pose.orientation.y = ddata[i*7+4];
        targ_pose.orientation.z = ddata[i*7+5];
        targ_pose.orientation.w = ddata[i*7+6];
        target_poses.push_back(targ_pose);
        cout<<"targ_pose "<<i+1<<":"<<targ_pose<<endl;
    }


    HandEyeCalibration cmc(num_cols,num_rows,dist,target_poses);

    return 0;
}
