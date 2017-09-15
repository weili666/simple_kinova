#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ctime>
#include <sensor_msgs/Image.h>
#include <wfov_camera_msgs/WFOVImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <ctime>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Pose.h>
#include <math.h>
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

using namespace std;
int num=0;
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
class ArmJointsSaved
{
public:
    ArmJointsSaved(ros::NodeHandle& nh):nh_(nh)
    {
        cout<<"begin save arm joints:"<<num<<endl;

        ros::Subscriber arm_sub = nh_.subscribe("/jaco_arm/joint_states",1,& ArmJointsSaved::ArmSavedCB,this);

        ros::Rate loop_rate(1);
        int num_loop = 2;
        while(num_loop)
        {
            cout<<"hello world!!"<<endl;
            ros::spinOnce();
            loop_rate.sleep();
            num_loop--;
        }
    }
    void ArmSavedCB(const sensor_msgs::JointState& msg);
private:
    ros::NodeHandle nh_;

};
void ArmJointsSaved::ArmSavedCB(const sensor_msgs::JointState& msg)
{
   double joint[6];

    double a,b,c,d;
    cout<<"accept arm joint"<<endl;

    cout<<msg.position.at(0)<<" "<<msg.position.at(1)<<" "<<msg.position.at(2)<<" "<<msg.position.at(3)<<" "<<msg.position.at(4)<<" "<<msg.position.at(5)<<endl;
    cout<<msg.position[0]<<" "<<msg.position[1]<<" "<<msg.position[2]<<" "<<msg.position[3]<<" "<<msg.position[4]<<" "<<msg.position[5]<<";"<<endl;

    joint[0] = msg.position[0]; joint[1] = msg.position[1]; joint[2] = msg.position[2];
    joint[3] = msg.position[3]; joint[4] = msg.position[4]; joint[5] = msg.position[5];


    ofstream file;
    file.open("/home/weili/simple_kinova/src/visual_servo/saved_image/grasp_pa.txt",ios::app|ios::out);

    cout<<"======================saved arm joints "<<num<<"======================="<<endl;
    file<<joint[0]<<" "<<joint[1]<<" "<<joint[2]<<" "<<joint[3]<<" "<<joint[4]<<" "<<joint[5]<<";"<<endl;

}
class PictureSaved
{
public:
    PictureSaved(ros::NodeHandle& nh):nh_(nh)
    {
        num++;
        cout<<"begin save picture: "<<num<<endl;
        ros::Subscriber pic_sub = nh_.subscribe("/camera/image_color", 1, & PictureSaved::PicSavedCB,this);

        ros::Rate loop_rate(1);
        int num_loop = 2;
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
    cv::Mat image_gray;
    cv::cvtColor(image,image_gray,CV_BGR2GRAY);

    char buff[100];
    sprintf(buff, "/home/weili/simple_kinova/src/visual_servo/saved_image/saved_%d.png",num);

    cout<<"======================saved picture num:"<<num<<"======================="<<endl;
    cout<<buff<<endl;
    cv::imwrite(buff, image_gray);

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointgrey_camera_picture");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);

    spinner.start();
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    ros::Rate loop_rate(10);

    planning_scene_monitor::PlanningSceneMonitorPtr scene_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    cout<<"hello world!"<<endl;
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();
    cout<<"scene set !"<<endl;
    moveit::planning_interface::MoveGroup robot_arm("arm");

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
    boost::shared_ptr<KDL::ChainIdSolver>        pose_to_jnt_solver;

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

    double a,b,c,d;
    q_.resize(6);
    q_(0)=-1.4735840227881702;q_(1)=2.919063173960516;q_(2)=1.0135101613037494;
    q_(3)=-2.0836849337533323;q_(4)=1.443466866652682;q_(5)=1.3149469735032022;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    cout<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<endl;
    x_.M.GetQuaternion(a,b,c,d);
    cout<<"qx:"<<a<<" qy:"<<b<<" qz:"<<c<<" qw:"<<d<<endl;

    //==========================================================================//

    //========================collect data for hand-eye-calibration=====================//
    ArmJoints arm(-1.4735840227881702,2.919063173960516,1.0135101613037494,-2.0836849337533323,1.443466866652682,1.3149469735032022);

    vector<ArmJoints> arm_joints;
    ArmJoints arm_joint_1(-0.2540991238232939, 2.8503408346632395, 1.5525089322215404, -2.0872549011945836, 1.4422766999615257, 0.968657734856853);
    arm_joints.push_back(arm_joint_1);

    ArmJoints arm_joint_2(-0.40521167776435263, 2.6548094169007497, 1.0972474694059202, -2.2395746660577753, 1.195947215460483, 0.968657734856853);
    arm_joints.push_back(arm_joint_2);

    ArmJoints arm_joint_3(-0.32724923474893686, 2.861794557879452, 1.20312217091132, -2.3526242564349844, 1.4696466722745425, 0.968657734856853);
    arm_joints.push_back(arm_joint_3);

    ArmJoints arm_joint_4(-0.2627618542171035, 3.2594023780994106, 1.20312217091132, -2.5620637666743042, 1.8361658151933518, 0.9412878957018904);
    arm_joints.push_back(arm_joint_4);

    ArmJoints arm_joint_5(-0.2627618542171035, 3.261038624273155, 0.7825106975645572, -2.7584135738397753, 2.06226526226388, 1.0162580109164931);
    arm_joints.push_back(arm_joint_5);

    ArmJoints arm_joint_6(-0.2627618542171035, 3.553108566286581, 0.7382357776000443, -2.9773728197116918, 2.5299340597030433, 1.0162580109164931);
    arm_joints.push_back(arm_joint_6);

    ArmJoints arm_joint_7(-0.4687365918782307, 3.816544200259475, 0.8094606227946413, 2.742943537383614, 3.0392529655363374, 1.0067377426516781);
    arm_joints.push_back(arm_joint_7);

    ArmJoints arm_joint_8(-0.9557600456975992, 3.8239073080413264, 0.991372701321493, 2.543024029093001, 2.5918139392115793, 1.5493765221473899);
    arm_joints.push_back(arm_joint_8);

    ArmJoints arm_joint_9(-1.0799224746714913, 3.995713156284518, 1.1521098528630702, 2.0313253225096117, 3.0059330918739198, 1.1638173753311678);
    arm_joints.push_back(arm_joint_9);

    ArmJoints arm_joint_10(-0.7882855626513123, 3.8288160465625602, 1.002922631232085, 2.725093700177358, 2.614423910550243, 1.4113368933653119);
    arm_joints.push_back(arm_joint_10);

    ArmJoints arm_joint_11(-1.3369095303808676, 3.5040211810742408, 0.633323542512599, 2.1895949887222033, 2.4525844100544547, 1.6064965338396267);
    arm_joints.push_back(arm_joint_11);

    ArmJoints arm_joint_12(-1.6169963127533729, 3.5539266893734536, 0.605411084285533, 1.7492958972799482, 2.6774936904338267, 1.9849157457733462);
    arm_joints.push_back(arm_joint_12);

    ArmJoints arm_joint_13(-2.007770195300079, 3.616922167062624, 0.6025236184526419, 0.8234781046132461, -3.0440128333526357, 1.6481462427595943);
    arm_joints.push_back(arm_joint_13);

    ArmJoints arm_joint_14(-2.318656968311569, 3.8378154005181564, 0.9432477825271072, 0.4938488693771327, 3.1106528469935797, 1.6481462427595943);
    arm_joints.push_back(arm_joint_14);

    ArmJoints arm_joint_15(-2.356194490192345, 3.962170109722752, 1.2656844854491889, 0.4938488693771327, 3.111842747368627, 1.6481462427595943);
    arm_joints.push_back(arm_joint_15);

    ArmJoints arm_joint_16(-2.134820023527835, 4.05216364927871, 1.3648218554498792, 0.642598533641018, 3.1285026841998356, 1.0174479112915407);
    arm_joints.push_back(arm_joint_16);

    ArmJoints arm_joint_17(-2.302294506574122, 3.8623590931243266, 1.5428838685678306, -0.09876977054647518, -2.447824275922047, 1.0174479112915407);
    arm_joints.push_back(arm_joint_17);

    ArmJoints arm_joint_18(-2.382181882425447, 3.8648134623849435, 1.8759081681406042, -0.3534291735288517, -2.234814531925367, 1.0174479112915407);
    arm_joints.push_back(arm_joint_18);

    ArmJoints arm_joint_19(-2.4707317223544725, 3.7478218609621985, 1.891308030302042, -0.3534291735288517, -2.234814531925367, 1.0174479112915407);
    arm_joints.push_back(arm_joint_19);

    ArmJoints arm_joint_20(-2.498644047423484, 3.526110504419794, 1.5399964693139667, -0.42006902072222735, -2.1884046888729927, 1.6183966028545367);
    arm_joints.push_back(arm_joint_20);

    ArmJoints arm_joint_21(-2.580456356110718, 3.4557519189487724, 1.5399964693139667, -0.46885893084080604, -2.1884046888729927, 1.463696770925142);
    arm_joints.push_back(arm_joint_21);

    ArmJoints arm_joint_22(-2.580456356110718, 3.4532975496881555, 1.8739831021466404, -0.5616787168140966, -2.1193848744819537, 1.531526418625024);
    arm_joints.push_back(arm_joint_22);

    ArmJoints arm_joint_23(-2.6805563256874176, 3.387847702738368, 1.9557954108338746, -0.5795286871784072, -2.1372349780043187, 1.531526418625024);
    arm_joints.push_back(arm_joint_23);

    ArmJoints arm_joint_24(-2.8181932845126747, 3.156318869153495, 2.0982451012230685, -0.5997585914508672, -2.1396147787544137, 1.531526418625024);
    arm_joints.push_back(arm_joint_24);

    ArmJoints arm_joint_25(-2.6324314068930317, 3.3780302256959, 1.6112212479295365, -0.49741883681838395, -2.1419948458206175, 1.531526418625024);
    arm_joints.push_back(arm_joint_25);

    ArmJoints arm_joint_26(-2.67670619369949, 3.2888548092268146, 1.6112212479295365, -0.49741883681838395, -2.1419948458206175, 1.531526418625024);
    arm_joints.push_back(arm_joint_26);

    ArmJoints arm_joint_27(-2.67670619369949, 3.2888548092268146, 2.028945255443408, -0.6568784699850502, -2.031325056193503, 1.7611963657690222);
    arm_joints.push_back(arm_joint_27);

    ArmJoints arm_joint_28(-2.6564938662902247, 3.1162308378967505, 2.112682630124606, -0.8437080754647333, -1.8790055576464209, 1.9908657802808012);
    arm_joints.push_back(arm_joint_28);

    ArmJoints arm_joint_29(-2.5660190935252896, 3.207860623626453, 2.241657258030217, -0.9567577989999974, -1.7564356990043954, 1.9908657802808012);
    arm_joints.push_back(arm_joint_29);

    ArmJoints arm_joint_30(-2.644943736642551, 3.2070425005395804, 2.438006798879579, -0.9174878908301249, -1.8349755153441407, 2.3097846471399706);
    arm_joints.push_back(arm_joint_30);

    ArmJoints arm_joint_31(-2.517894174730904, 3.145683269024155, 2.5746815576029904, -1.1293073349775937, -1.8349755153441407, 2.3097846471399706);
    arm_joints.push_back(arm_joint_31);

    ArmJoints arm_joint_32(-2.565056360791226, 3.145683269024155, 2.724831112493876, -1.1293073349775937, -1.8349755153441407, 2.429974705031901);
    arm_joints.push_back(arm_joint_32);

    ArmJoints arm_joint_33(-2.392769279339057, 3.0998683761593036, 2.453406794199071, -1.3137569399701003, -1.6243461047297734, 2.689394292080955);
    arm_joints.push_back(arm_joint_33);

    ArmJoints arm_joint_34(-2.4149068059003405, 3.191498161889006, 2.982780634621207, -1.2959069696057897, -1.899235461918881, -3.082092308515243);
    arm_joints.push_back(arm_joint_34);

    ArmJoints arm_joint_35(-2.299407107320258, 3.193134408062751, 3.31772986702989, -1.3673068510630322, -2.1134348399744987, -2.6584536865364132);
    arm_joints.push_back(arm_joint_35);

    ArmJoints arm_joint_36(-2.4149068059003405, 3.0147835751245804, 3.156992648909285, -1.2959069696057897, -2.183644821056694, -2.469243547937337);
    arm_joints.push_back(arm_joint_36);

    ArmJoints arm_joint_37(-3.0925052683774523, 2.644173816771409, 2.9943304979527716, -0.8841679505886803, -2.0682151636132806, -2.469243547937337);
    arm_joints.push_back(arm_joint_37);

    ArmJoints arm_joint_38(-2.5304065044804234, 2.727622371632388, 2.9943304979527716, -1.274487031800228, -2.1967347904466514, -3.0285425305803653);
    arm_joints.push_back(arm_joint_38);

    ArmJoints arm_joint_39(-1.1270849270479228, 2.6760806171594305, 2.3764070839177194, -2.7298535679936564, -1.5731761275449916, -2.9488128138655725);
    arm_joints.push_back(arm_joint_39);

    ArmJoints arm_joint_40(-0.7805858313076746, 3.0319641599488993, 2.374481884765701, 2.8678931967757335, -1.2459268927960547, -2.8012534494508987);
    arm_joints.push_back(arm_joint_40);

    ArmJoints arm_joint_41(-0.5774984929004123, 3.2716742244024957, 2.374481884765701, 2.4192642700759284, -1.3197065750033916, -2.17293491873294);
    arm_joints.push_back(arm_joint_41);

    ArmJoints arm_joint_42(-0.40521167776435263, 3.2716742244024957, 1.997182798386469, 2.080114966312081, -1.4291864642554595, -2.1884046888729927);
    arm_joints.push_back(arm_joint_42);

    ArmJoints arm_joint_43(-0.43504920198538244, 3.270856101315623, 1.6429837049864755, 1.8373555824103445, -1.5184364492350664, -1.5779357290451799);
    arm_joints.push_back(arm_joint_43);

    ArmJoints arm_joint_44(-0.1588122852846947, 3.2700379782287508, 1.171359713854381, 2.008715084854839, -2.0444150255834606, -1.2792462338262531);
    arm_joints.push_back(arm_joint_44);

    ArmJoints arm_joint_45(-0.2560245892914219, 3.509748042682347, 1.2079346361591476, 2.006335284104744, -2.1455648132618688, -1.2233166551412822);
    arm_joints.push_back(arm_joint_45);

    ArmJoints arm_joint_46(-0.3618990910597395, 3.59810533606456, 1.2079346361591476, 1.9325553355812977, -2.177694786549239, -0.28916922695411174);
    arm_joints.push_back(arm_joint_46);

    ArmJoints arm_joint_47(-0.5332239724100631, 3.59810533606456, 1.2021597044933654, 1.7421558292393913, -2.0682151636132806, -0.214199111739509);
    arm_joints.push_back(arm_joint_47);

    ArmJoints arm_joint_48(-0.5842362904583132, 3.789546138392688, 1.089547471746174, 1.9028054293601322, -2.4609142453120043, -0.0333193410301984);
    arm_joints.push_back(arm_joint_48);

    ArmJoints arm_joint_49(-0.6102234163753062, 3.9098102321629224, 1.089547471746174, 2.301454545566311, -2.9238230417768145, 0.7770688607721166);
    arm_joints.push_back(arm_joint_49);

    ArmJoints arm_joint_50(-0.9047477276493492, 3.863177216211199, 1.1213098622240858, 2.626323713249044, 2.6191840446826506, 1.5446163880149815);
    arm_joints.push_back(arm_joint_50);

    ArmJoints arm_joint_51(-1.0731852097458088, 3.941717032550944, 1.1184223298121674, 2.2479045013153245, 2.813153518465809, 1.5446163880149815);
    arm_joints.push_back(arm_joint_51);

    ArmJoints arm_joint_52(-0.32243690265916314, 3.8525416160818584, 2.247432322854054, -2.5656340004316642, 2.000385382755344, 0.9174877576720704);
    arm_joints.push_back(arm_joint_52);

    ArmJoints arm_joint_53(0.556323699073193, 2.863430804053197, 3.133892655930047, -2.418074369700881, 1.9194656325074497, -0.8972573207673928);
    arm_joints.push_back(arm_joint_53);

    ArmJoints arm_joint_54(0.688185859390722, 2.48382169174443, 4.407277272197037, -1.926605567389952, 2.1027252039668545, -2.846472859496009);
    arm_joints.push_back(arm_joint_54);

    ArmJoints arm_joint_55(0.688185859390722, 2.4821854455706855, 5.034825883043854, -2.216964827877166, 2.514464289562991, 2.2490949343225903);
    arm_joints.push_back(arm_joint_55);

    ArmJoints arm_joint_56(0.688185859390722, 2.4821854455706855, 4.040565582731415, -1.9456455712873646, 1.9599254410523694, -2.9095429056957);
    arm_joints.push_back(arm_joint_56);

    ArmJoints arm_joint_57(2.2310695948004984, 2.5517259079548347, 5.302400069350731, -1.7005061203194238, -2.3026447122574676, 0.2261002460188557);
    arm_joints.push_back(arm_joint_57);

    ArmJoints arm_joint_58(-3.041492950329202, 3.4745687499468363, 5.882786494137225, -0.973417735831206, -2.2824148079850075, -0.9508070987022688);
    arm_joints.push_back(arm_joint_58);

    ArmJoints arm_joint_59(2.359081889446209, 3.536746104549134, 5.884711426973134, -2.0527452603151737, -1.424426729597215, -0.5414483464884459);
    arm_joints.push_back(arm_joint_59);

    ArmJoints arm_joint_60(1.660308899457986, 3.234040562406368, 5.338012392079488, -2.930963242975426, -0.9972176741239444, -0.1427997629144837);
    arm_joints.push_back(arm_joint_60);

    vector<ArmJoints>::iterator aj_iter;
    for(aj_iter=arm_joints.begin();aj_iter!=arm_joints.end();aj_iter++)
    {
        ArmJoints aj=*aj_iter;

        double a,b,c,d;

        std::map<std::string,double> joint;
        joint["jaco_joint_1"]=aj.joint1;
        joint["jaco_joint_2"]=aj.joint2;
        joint["jaco_joint_3"]=aj.joint3;
        joint["jaco_joint_4"]=aj.joint4;
        joint["jaco_joint_5"]=aj.joint5;
        joint["jaco_joint_6"]=aj.joint6;

        q_(0)=aj.joint1;q_(1)=aj.joint2;q_(2)=aj.joint3;
        q_(3)=aj.joint4;q_(4)=aj.joint5;q_(5)=aj.joint6;

        jnt_to_pose_solver_->JntToCart(q_, x_);
        cout<<x_.p.x()<<" "<<x_.p.y()<<" "<<x_.p.z();

        x_.M.GetQuaternion(a,b,c,d);
        cout<<" "<<a<<" "<<b<<" "<<c<<" "<<d<<endl;

        ofstream file;
        file.open("/home/weili/simple_kinova/src/visual_servo/saved_image/grasp_pa.txt",ios::app|ios::out);
//        file<<q_(0)<<" "<<q_(1)<<" "<<q_(2)<<" "<<q_(3)<<" "<<q_(4)<<" "<<q_(5)<<";"<<endl;
        file<<x_.p.x()<<" "<<x_.p.y()<<" "<<x_.p.z()<<" "<<a<<" "<<b<<" "<<c<<" "<<d<<";"<<endl;

        cv::waitKey(2.0);

        moveit::planning_interface::MoveGroup::Plan my_plan;
        robot_arm.setJointValueTarget(joint);
        robot_arm.setPlanningTime(12.0);
        robot_arm.setPlannerId("RRTstarkConfigDefault");
        bool success_=robot_arm.plan(my_plan);
        sleep(3.0);
        ROS_INFO("Visualizing simultanious plan 1st stage %s",success_?"":"FAILED");
        robot_arm.execute(my_plan);
        sleep(3.0);
        PictureSaved ps(nh);
//        ArmJointsSaved as(nh);
    }

    //==================================================================================//
    ROS_INFO("Finished");
    ros::spin();
    return 0;
}
