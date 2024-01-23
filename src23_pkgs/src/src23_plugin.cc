/*

Kubot package simulation code

kudos edu.ver

23.03.16

*/

//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>
#include <time.h>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>   //model plugin gazebo API에서 확인
#include <gazebo/physics/physics.hh> //우리는 ODE physics 사용
#include <gazebo/sensors/sensors.hh> //IMU sensor 사용

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> //topic을 어떤 형태로 보내는지

#include <functional>
#include <ignition/math/Vector3.hh>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <src23_pkgs/Kubot_control_msgs.h>

// KUDOS CKubot Header 파일 선언
#include "kudos/CKubot.h"
// #include "kudos/kinematics.h"

//* Print color
#define C_BLACK "\033[30m"
#define C_RED "\x1b[91m"
#define C_GREEN "\x1b[92m"
#define C_YELLOW "\x1b[93m"
#define C_BLUE "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN "\x1b[96m"
#define C_RESET "\x1b[0m"

using namespace std;
// 여기까지가 책으로 따지자면 목차. namespace까지

// Eigen//
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::VectorXd;
using Eigen::VectorXf;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
// Eigen 파일 안에 있는 namespace를 사용하겠다.

VectorXd test_vector(12);

namespace gazebo
{

    class src23_plugin : public ModelPlugin
    {
        //*** Variables for Kubot Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        physics::ModelPtr model; // model = _model 관련

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        //* Index setting for each joint
        // physics::JointPtr L_Foot_joint;
        // physics::JointPtr R_Foot_joint;

        //* ROS Subscribe
        ros::NodeHandle nh; // ros 통신을 위한 node handle 선언
        ros::Subscriber KubotModesp;
        int ControlMode_by_ROS = 1;

        // FOOTSTEP
        int StartFootCheck;
        bool FSW_STOP;

        // ZMP
        std_msgs::Float64 m_ZMP_x;
        std_msgs::Float64 m_ZMP_y;
        std_msgs::Float64 m_ZMPcontrol_x;
        std_msgs::Float64 m_ZMPcontrol_y;
        std_msgs::Float64 m_ZMP_x_l_margin;
        std_msgs::Float64 m_ZMP_x_u_margin;
        std_msgs::Float64 m_ZMP_y_l_margin;
        std_msgs::Float64 m_ZMP_y_u_margin;

        std_msgs::Float64 m_Base_refpos_x;
        std_msgs::Float64 m_Base_refpos_y;
        std_msgs::Float64 m_Base_refpos_z;

        // Preview
        std_msgs::Float64 m_preview_ref_ZMP_x;
        std_msgs::Float64 m_preview_ref_ZMP_y;
        std_msgs::Float64 m_preview_COM_x;
        std_msgs::Float64 m_preview_COM_y;

        std_msgs::Float64 m_L_foot_ref_x;
        std_msgs::Float64 m_L_foot_ref_z;
        std_msgs::Float64 m_L_foot_FK_x;
        std_msgs::Float64 m_L_foot_FK_z;

        std_msgs::Float64 m_R_foot_ref_x;
        std_msgs::Float64 m_R_foot_ref_z;
        std_msgs::Float64 m_R_foot_FK_x;
        std_msgs::Float64 m_R_foot_FK_z;

        std_msgs::Float64 m_preview_FK_x;
        std_msgs::Float64 m_preview_FK_y; /// 박사님 과제 FK

        ////////////////////////////////
        std_msgs::Float64 m_time_sec;
        std_msgs::Float64 m_time_old_sec;
        std_msgs::Float64 m_time_init_sec;
        std_msgs::Float64 m_time_wsec;

        // std_msgs::Float64 m_predicted_time;
        // std_msgs::Float64 m_predicted_step;

        //* ROS Publish
        ros::Publisher KubotModesp_pub;
        // ros::Publisher LHY_pub;
        // ros::Publisher LHR_pub;
        // ros::Publisher LHP_pub;
        // ros::Publisher LKN_pub;
        // ros::Publisher LAP_pub;
        // ros::Publisher LAR_pub;

        ros::Publisher COM_x;
        ros::Publisher COM_y;
        ros::Publisher COM_z;

        // rqt (Walking)
        ros::Publisher P_ZMP_x;
        ros::Publisher P_ZMP_y;
        ros::Publisher P_ZMPcontrol_x;
        ros::Publisher P_ZMPcontrol_y;
        ros::Publisher P_preview_ref_ZMP_x;
        ros::Publisher P_preview_ref_ZMP_y;
        ros::Publisher P_preview_COM_x;
        ros::Publisher P_preview_COM_y;

        ros::Publisher P_preview_FK_x;
        ros::Publisher P_preview_FK_y; /// 박사님 과제 FK

        ros::Publisher P_L_foot_ref_x;
        ros::Publisher P_L_foot_ref_z;
        ros::Publisher P_L_foot_FK_x;
        ros::Publisher P_L_foot_FK_z;

        ros::Publisher P_R_foot_ref_x;
        ros::Publisher P_R_foot_ref_z;
        ros::Publisher P_R_foot_FK_x;
        ros::Publisher P_R_foot_FK_z;

        ros::Publisher P_Base_refpos_x;
        ros::Publisher P_Base_refpos_y;
        ros::Publisher P_Base_refpos_z;

        /////////////////////////////////////
        ros::Publisher P_time_sec;
        ros::Publisher P_time_old_sec;
        ros::Publisher P_time_init_sec;
        ros::Publisher P_time_wsec;

        // ros::Publisher P_predicted_time;
        // ros::Publisher P_predicted_step;

        std_msgs::Float64 KubotModesp_msg;
        // std_msgs::Float64 LHY_msg;
        // std_msgs::Float64 LHR_msg;
        // std_msgs::Float64 LHP_msg;
        // std_msgs::Float64 LKN_msg;
        // std_msgs::Float64 LAP_msg;
        // std_msgs::Float64 LAR_msg;

        ros::Subscriber Kubot_control_mode_sub;
        ros::Subscriber Kubot_joy_sub;

        ros::Publisher cmd_vel_pub_;
        ros::Subscriber cmd_vel_sub;
        std::string joy_topic_;
        std::string cmd_vel_topic_;

        geometry_msgs::Twist cmd_vel_msg;

        enum
        { // wst는 torso joint 같음
            LHY = 0,
            LHR,
            LHP,
            LKN,
            LAP,
            LAR,
            RHY,
            RHR,
            RHP,
            RKN,
            RAP,
            RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot
                  // 구조체는 변수 집합이라고 생각하면 편하다.

        // int Q0 = Kubot.Q0;
        // int Qi = Kubot.Qi;
        // int Qj = Kubot.Qj;
        // int Qk = Kubot.Qk;

        typedef struct RobotJoint // Joint variable struct for joint control
        {
            double targetDegree; // The target deg, [deg]
            double targetRadian; // The target rad, [rad]
            double init_targetradian;

            double targetRadian_interpolation; // The interpolated target rad, [rad]

            double targetVelocity; // The target vel, [rad/s]
            double targetTorque;   // The target torque, [N·m]

            double actualDegree;   // The actual deg, [deg]
            double actualRadian;   // The actual rad, [rad]
            double actualVelocity; // The actual vel, [rad/s]
            double actualRPM;      // The actual rpm of input stage, [rpm]
            double actualTorque;   // The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT *joint;

        bool joint_by_algorithm_update = false;

        double zmpFK_X;
        double zmpFK_Y;
        double zmpFK_Z;

        double L_foot_ref_x;
        double L_foot_ref_y;
        double L_foot_ref_z;

        double R_foot_ref_x;
        double R_foot_ref_y;
        double R_foot_ref_z;

        double L_foot_FK_x;
        double L_foot_FK_y;
        double L_foot_FK_z;

        double R_foot_FK_x;
        double R_foot_FK_y;
        double R_foot_FK_z;

        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU; // imu 센서 추가

        CKubot Kubot; // Kubot이라는 이름을 가진 CKubot class 선언함과 동시에 변수들을 다 갖고옴

        //* ROS Subscribe
        // ros::NodeHandle nh;
        // ros::Subscriber RoS_Mode;
        // ros::Subscriber Save_Mode;
        // int ControlMode_by_ROS = 0;
        // bool Balancing_Control_Mode_by_ROS = 0;

    public:
        //*** Functions for Kubot Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation
        void UpdateAlgorithm();                                        // Algorithm update while simulation

        void jointUpdateByAlgorithm(); // joint ref update by control algorithm

        void setjoints();    // Get each joint data from [physics::ModelPtr _model]
        void getjointdata(); // Get encoder data of each joint

        void setsensor();
        void getsensordata();

        void jointcontroller();

        void initializejoint(); // 추가한
        void setjointPIDgain(); // 함수

        //*ROS subscribe
        // void ros_MODE(const std_msgs::Int32 &msg);
        void KubotMode(const std_msgs::Int32 &msg);
        void Kubot_control_callback(const src23_pkgs::Kubot_control_msgs &msg);
        // void Kubot_joy_callback(const sensor_msgs::Joy &joy_msgs);
        void Kubot_joy_cmd_vel(const sensor_msgs::Joy &joy_msgs);
        void Convert_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg);

        void InitRosTopic();
        void RosTopicPublish();

        // void Compensate();
    };
    GZ_REGISTER_MODEL_PLUGIN(src23_plugin);

}

// model.sdf파일에 대한 정보를 불러오는 함수
void gazebo::src23_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{

    printf("\nLoad_do\n");

    //* ROS Subscribe
    KubotModesp = nh.subscribe("/KubotMode", 1, &gazebo::src23_plugin::KubotMode, this);
    Kubot_control_mode_sub = nh.subscribe("/Kubot_Control_Msg", 1, &gazebo::src23_plugin::Kubot_control_callback, this);
    // Kubot_joy_sub = nh.subscribe("/joy",1,&gazebo::src23_plugin::Kubot_joy_callback,this);
    Kubot_joy_sub = nh.subscribe("/joy", 1, &gazebo::src23_plugin::Kubot_joy_cmd_vel, this);
    cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &gazebo::src23_plugin::Convert_cmd_vel, this);

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    setjoints();
    setsensor();

    InitRosTopic();

    nDoF = 12;                    // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    initializejoint();
    setjointPIDgain();

    Kubot.initializeBkubot();

    Kubot.setWalkingReadyPos(0, 0, 0.43);

    //* setting for getting dt

    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&src23_plugin::UpdateAlgorithm, this));
}

// void gazebo::src23_plugin::Kubot_joy_callback(const sensor_msgs::Joy &joy_msgs){
//     printf("stamp : %d\n",joy_msgs.header.seq);
//     printf("joy_data[0] :%f\n",joy_msgs.axes[0]);
//     printf("joy_data[1] :%f\n",joy_msgs.axes[1]);
//     printf("joy_data[2] :%f\n",joy_msgs.axes[2]);

//     if(joy_msgs.buttons[2] == 1){ // 세모 : 14번
//         Kubot.startTime = Kubot.realTime;
//         Kubot.ControlMode = CTRLMODE_WALKING_TEST;
//         Kubot.CommandFlag = ACT_START_WALK;
//         printf("INF WALKTEST START! \n");
//         Kubot.zmp.previewControl.count = 0;

//         Kubot.Move_current = true;
//         ControlMode_by_ROS = 0;

//         Kubot.WALK_READY = true;
//         Kubot.WALK_ING = false;
//     }
//     if(joy_msgs.buttons[1] == 1){ // O : 2번
//         Kubot.startTime = Kubot.realTime;
//         Kubot.ControlMode = CTRLMODE_WALKREADY;
//         Kubot.CommandFlag = GOTO_WALK_READY_POS;
//         printf("WALK READY START! \n");
//     }
//     if(Kubot.Move_current == true){
//     Kubot.FBSize_msgs = joy_msgs.axes[1]*0.05;
//     Kubot.LRSize_msgs = joy_msgs.axes[3]*0.03;
//     Kubot.TurnSize_msgs = joy_msgs.axes[0]*0.15;
//     Kubot.stop_msgs = joy_msgs.buttons[0];
//     }

// }

void gazebo::src23_plugin::Kubot_joy_cmd_vel(const sensor_msgs::Joy &joy_msgs)
{
    // printf("stamp : %d\n",joy_msgs.header.seq);
    // printf("joy_data[0] :%f\n",joy_msgs.axes[0]);
    // printf("joy_data[1] :%f\n",joy_msgs.axes[1]); //
    // printf("joy_data[2] :%f\n",joy_msgs.axes[2]);

    // geometry_msgs::Twist cmd_vel_msg;

    if (joy_msgs.buttons[2] == 1)
    { // 세모 : 14번
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKING_TEST;
        Kubot.CommandFlag = ACT_START_WALK;
        printf("INF WALKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.WALK_READY = true;
        Kubot.WALK_ING = false;
    }
    if (joy_msgs.buttons[1] == 1)
    { // O : 2번
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");
    }
    if (Kubot.Move_current == true)
    {

        // cmd_vel_msg.linear.x = joy_msgs.axes[1]*0.05;
        // cmd_vel_msg.linear.y = joy_msgs.axes[3]*0.03;

        // cmd_vel_msg.angular.z = joy_msgs.axes[0]*0.15;

        cmd_vel_msg.linear.x = joy_msgs.axes[1] * 0.05;
        cmd_vel_msg.linear.y = joy_msgs.axes[0] * 0.03;
        cmd_vel_msg.angular.z = joy_msgs.axes[3] * 0.15;

        // Kubot.FBSize_msgs = joy_msgs.axes[1]*0.05;
        // Kubot.LRSize_msgs = joy_msgs.axes[3]*0.03;
        // Kubot.TurnSize_msgs = joy_msgs.axes[0]*0.15;
        Kubot.stop_msgs = joy_msgs.buttons[0];
    }
    // cmd_vel_pub_.publish(cmd_vel_msg);
    cout << "cmd_vel : \n"
         << cmd_vel_msg << endl;
}

void gazebo::src23_plugin::Convert_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg)
{

    cmd_vel_msg = *msg;

    Kubot.FBSize_msgs = cmd_vel_msg.linear.x;
    Kubot.LRSize_msgs = cmd_vel_msg.linear.y;
    Kubot.TurnSize_msgs = cmd_vel_msg.angular.z;

    // Kubot.FBSize_msgs = twist.linear.x;
    // Kubot.LRSize_msgs = twist.linear.y;
    // Kubot.TurnSize_msgs = twist.angular.z;
}

void gazebo::src23_plugin::KubotMode(const std_msgs::Int32 &msg)
{
    // ROS_INFO("I heard: [%d]",msg.data);

    ControlMode_by_ROS = msg.data;

    switch (ControlMode_by_ROS)
    {

    case 1:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_HOMEPOSE;
        Kubot.CommandFlag = RETURN_HOMEPOSE;
        printf("Kubot home pose! \n");

        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        break;

    case 2:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");

        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        break;

    case 3:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_UPDOWN;
        Kubot.CommandFlag = UPDOWN;
        printf("Up Down START! \n");

        Kubot.updown_cnt = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        break;

    case 4:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_SWAY;
        Kubot.CommandFlag = SWAYMOTION;
        printf("SWAY START! \n");
        Kubot.zmp.previewControl.count = 0;
        Kubot.sway();

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.SWAY_READY = true;
        Kubot.SWAY_ING = false;

        break;

    case 5:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_KICKTEST;
        Kubot.CommandFlag = KICK_PATTERN;
        printf("KICKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.DSP_baseRef();

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.KICK_READY = true;
        Kubot.KICK_ING = false;

        break;

    case 11:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKING_TEST;
        Kubot.CommandFlag = ACT_PATTERN_TEST;
        printf("WALKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.WALK_READY = true;
        Kubot.WALK_ING = false;

        break;

    case 12:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKING_TEST;
        Kubot.CommandFlag = ACT_SIDE_PATTERN_TEST;
        printf("SIDE WALKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.WALK_READY = true;
        Kubot.WALK_ING = false;

        break;

    case 13:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKING_TEST;
        Kubot.CommandFlag = ACT_TURN_PATTERN_TEST;
        printf("TURN WALKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.WALK_READY = true;
        Kubot.WALK_ING = false;

        break;

    case 14:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKING_TEST;
        Kubot.CommandFlag = ACT_START_WALK;
        printf("INF WALKTEST START! \n");
        Kubot.zmp.previewControl.count = 0;

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.WALK_READY = true;
        Kubot.WALK_ING = false;

        break;

    case 23:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_MOTION;
        Kubot.CommandFlag = ACT_HURDLE;
        printf("HURDLE START! \n");

        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.HURDLE_ING = false;

        break;

    case 99: // save file

        printf("save file START!\n");
        Kubot.save_file(50000);
        printf("save file DONE!\n");

        break;

    default:
        break;
    }
}

void gazebo::src23_plugin::Kubot_control_callback(const src23_pkgs::Kubot_control_msgs &msg)
{
    int Kubot_mode_msgs;

    // Kubot_mode_msgs = msg.Kubot_Mode;
    Kubot.steps_msgs = msg.steps;
    Kubot.FBSize_msgs = msg.FBSize;
    Kubot.LRSize_msgs = msg.LRSize;
    Kubot.TurnSize_msgs = msg.TurnSize;
    Kubot.footHeight_msgs = msg.footHeight;
    Kubot.stop_msgs = msg.stop;
    printf("======Subscriber_read_data=====\n");
    // printf("Kubot_mode : %d, steps : %d, FBSize : %lf, LRSize : %lf, TurnSize : %lf, footHeight : %lf\n" , Kubot_mode_msgs, Kubot.steps_msgs, Kubot.FBSize_msgs, Kubot.LRSize_msgs, Kubot.TurnSize_msgs, Kubot.footHeight_msgs);
    printf("steps : %d, FBSize : %lf, LRSize : %lf, TurnSize : %lf, footHeight : %lf, stop : %d", Kubot.steps_msgs, Kubot.FBSize_msgs, Kubot.LRSize_msgs, Kubot.TurnSize_msgs, Kubot.footHeight_msgs, Kubot.stop_msgs);

    // ControlMode_by_ROS = Kubot_mode_msgs;

    // switch (ControlMode_by_ROS) {

    // case 1:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_HOMEPOSE;
    //     Kubot.CommandFlag = RETURN_HOMEPOSE;
    //     printf("Kubot home pose! \n");

    //     Kubot.Move_current = false;
    //     ControlMode_by_ROS = 0;

    //     break;

    // case 2:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKREADY;
    //     Kubot.CommandFlag = GOTO_WALK_READY_POS;
    //     printf("WALK READY START! \n");

    //     Kubot.Move_current = false;
    //     ControlMode_by_ROS = 0;

    //     break;

    // case 3:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_UPDOWN;
    //     Kubot.CommandFlag = UPDOWN;
    //     printf("Up Down START! \n");

    //     Kubot.updown_cnt = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     break;

    // case 4:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_SWAY;
    //     Kubot.CommandFlag = SWAYMOTION_PATTERN;
    //     printf("SWAY START! \n");
    //     Kubot.zmp.previewControl.count = 0;
    //     Kubot.sway();

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.SWAY_READY = true;
    //     Kubot.SWAY_ING = false;

    //     break;

    // case 5:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_KICKTEST;
    //     Kubot.CommandFlag = KICK_PATTERN;
    //     printf("KICKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.DSP_baseRef();

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.KICK_READY = true;
    //     Kubot.KICK_ING = false;

    //     break;

    // case 11:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;
    //     Kubot.CommandFlag = ACT_PATTERN_TEST;
    //     printf("WALKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.WALK_READY = true;
    //     Kubot.WALK_ING = false;

    //     break;

    // case 12:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;
    //     Kubot.CommandFlag = ACT_SIDE_PATTERN_TEST;
    //     printf("SIDE WALKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.WALK_READY = true;
    //     Kubot.WALK_ING = false;

    //     break;

    // case 13:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;
    //     Kubot.CommandFlag = ACT_TURN_PATTERN_TEST;
    //     printf("TURN WALKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.WALK_READY = true;
    //     Kubot.WALK_ING = false;

    //     break;

    // case 14:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;
    //     Kubot.CommandFlag = ACT_START_WALK;
    //     printf("INF WALKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.WALK_READY = true;
    //     Kubot.WALK_ING = false;

    //     break;

    // case 15:
    //     Kubot.startTime = Kubot.realTime;
    //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;
    //     Kubot.CommandFlag = ACT_STOPPING_PATTERN_WALK;
    //     printf("STOPPING WALKTEST START! \n");
    //     Kubot.zmp.previewControl.count = 0;

    //     Kubot.Move_current = true;
    //     ControlMode_by_ROS = 0;

    //     Kubot.WALK_READY = true;
    //     Kubot.WALK_ING = false;

    //     break;

    // case 99://save file

    //     printf("save file START!\n");
    //     Kubot.save_file(50000);
    //     printf("save file DONE!\n");

    //     break;

    // default:
    //      break;
    // }
}

void gazebo::src23_plugin::UpdateAlgorithm()
{

    //* UPDATE TIME : 1ms
    // printf("update_do_time\n");
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //        cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;
    Kubot.realTime = Kubot.realTime + dt;
    //* setting for getting dt at next step
    last_update_time = current_time;

    getjointdata();
    getsensordata();

    // if(Kubot.CommandFlag == ACT_START_WALK || Kubot.CommandFlag == ACT_INF_WALK){

    //     cmd_vel_sub = nh.subscribe("/cmd_vel", 1000 , &gazebo::src23_plugin::Covert_cmd_vel,this);
    //     cout << "cmd_vel_sub : \n" << cmd_vel_sub << endl;
    // }

    // cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &gazebo::src23_plugin::Covert_cmd_vel, this);
    // Covert_cmd_vel(cmd_vel_msg);

    static double steps = 0;
    static double FBSize = 0;
    static double LRSize = 0;
    static double TurnSize = 0;

    VectorXd L_foot_FKcheck(6);
    VectorXd R_foot_FKcheck(6);

    static int con_count = 0;

    //* Real or simulated real-time thread time setting
    if (con_count % (int)(tasktime * 1000 + 0.001) == 0)
    {

        //* ControlMode
        switch (Kubot.ControlMode)
        {

        case CTRLMODE_HOMEPOSE:

            break;

        case CTRLMODE_WALKREADY:

            break;

        case CTRLMODE_UPDOWN:

            break;

        case CTRLMODE_SWAY:

            break;

        case CTRLMODE_KICKTEST:

            break;

        case CTRLMODE_WALKING_TEST:

            break;

        case CTRLMODE_WALKING:

            break;

            // case ACT_INF_WALK:

            // break;
        }

        //* CommandFlag
        switch (Kubot.CommandFlag)
        {

        case GOTO_WALK_READY_POS: // down
            // WalkReady : Preparing the robot for walking
            if (Kubot.Move_current == false)
            {
                Kubot.walkingReady(5.0);
                for (int j = 0; j < nDoF; j++)
                {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("WALK_READY COMPLETE !\n");
            }
            break;

        case RETURN_HOMEPOSE: // up

            if (Kubot.Move_current == false)
            {

                Kubot.HomePose(5.0);
                for (int j = 0; j < nDoF; j++)
                {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }
            }
            else
            {
                printf("RETURN_HOMEPOSE COMPLETE !\n");
                Kubot.ControlMode = CTRLMODE_HOMEPOSE;
                Kubot.CommandFlag = NONE_ACT;
            }
            break;

        case UPDOWN: // up and down

            if (Kubot.Move_current == true)
            {
                Kubot.ControlMode = CTRLMODE_UPDOWN;
                Kubot.CommandFlag = UPDOWN;
                printf("UPDOWN ING !\n");
            }

            break;

        case SWAYMOTION_PATTERN: // sway motion pattern
            printf("pattern_start=====\n");
            if (Kubot.SWAY_ING == false)
            {
                Kubot.ControlMode = CTRLMODE_SWAY;
                printf("sway_start=====\n");

                printf("SWAY READY !\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("PATTERN READY COMPLETE !\n");
            }
            break;

        case SWAYMOTION: // sway motion pattern

            if (Kubot.SWAY_READY = true)
            {
                Kubot.ControlMode = CTRLMODE_SWAY;

                // printf("SWAY ING !\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("SWAY COMPLETE !\n");
            }

            break;

        case KICK_PATTERN: // kick motion pattern
            printf("=====pattern_start=====\n");
            if (Kubot.KICK_ING == false)
            {
                Kubot.ControlMode = CTRLMODE_KICKTEST;
                printf("=====KICK START=====\n");

                printf("=====KICK READY !=====\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break;

        case KICKTEST: // sway motion pattern

            if (Kubot.KICK_READY = true)
            {
                Kubot.ControlMode = CTRLMODE_KICKTEST;

                // printf("KICK ING !\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====KICK COMPLETE !=====\n");
            }
            break;

        case ACT_HURDLE:

            if (Kubot.HURDLE_ING == false)
            {
                Kubot.hurdle(5.0);
            }
            else
            {
                Kubot.HURDLE_ING = false;
                Kubot.CommandFlag = NONE_ACT;
                printf("=====HURDLE COMPLETE !=====\n");
            }
            break;

        case ACT_PATTERN_TEST: // kick motion pattern
            printf("=====WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false)
            {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====WALK READY !=====\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break;

        case ACT_SIDE_PATTERN_TEST: // side motion pattern
            printf("=====SIDE WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false)
            {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====SIDE WALK READY !=====\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break;

        case ACT_TURN_PATTERN_TEST: // side motion pattern
            printf("=====TURN WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false)
            {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====TURN WALK READY !=====\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====TURN READY COMPLETE !=====\n");
            }
            break;

        case ACT_TEST_WALK: // sway motion pattern

            if (Kubot.WALK_READY = true)
            {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                // printf("KICK ING !\n");
            }
            else
            {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====WALK COMPLETE !=====\n");
            }
            break;

            // case ACT_STOPPING_PATTERN_WALK:

            //     break;

        case ACT_STOPPING_WALK:
            // if (Kubot.WALK_READY = true) {
            //     Kubot.ControlMode = CTRLMODE_WALKING_TEST;

            //     // printf("KICK ING !\n");
            // }
            // else {
            //     Kubot.CommandFlag = NONE_ACT;
            //     printf("=====WALK COMPLETE !=====\n");
            // }
            break;

        case ACT_STOP_WALK:

            break;

        case ACT_INF_WALK:

            break;

        case NONE_ACT:
            FBSize = 0;
            LRSize = 0;
            TurnSize = 0;

            break;

        default:

            break;
        }

        Kubot.walkingPatternGenerator(Kubot.ControlMode, Kubot.CommandFlag, Kubot.steps_msgs, Kubot.FBSize_msgs, Kubot.LRSize_msgs, Kubot.TurnSize_msgs, Kubot.footHeight_msgs, Kubot.stop_msgs);

        if (Kubot.ControlMode != CTRLMODE_WALKREADY)
        {

            VectorXd joint_IK(12);

            if (Kubot.Move_current == true && Kubot.CommandFlag == UPDOWN)
            {

                VectorXd GFL_Vector(6);
                GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                VectorXd GFR_Vector(6);
                GFR_Vector << 0, -0.05, 0, 0, 0, 0;

                VectorXd GB_Vector(6);

                double ref_B_x = 0;
                double ref_B_y = 0;
                double ref_B_z; // Base 위치

                double z_max = 0.48;
                double z_min = 0.40;
                double T = 1000;

                ref_B_z = Kubot.cosWave(z_max - z_min, T, Kubot.updown_cnt, z_min);
                Kubot.updown_cnt = Kubot.updown_cnt + 1;

                std::cout << "\nref_B_z:\n"
                          << ref_B_z << std::endl;

                GB_Vector << ref_B_x, ref_B_y, ref_B_z, 0, 0, 0;

                joint_IK << Kubot.Geometric_IK_L(GB_Vector, GFL_Vector), Kubot.Geometric_IK_R(GB_Vector, GFR_Vector);
            }

            else if (Kubot.CommandFlag == SWAYMOTION && Kubot.SWAY_READY == true)
            {

                VectorXd GBS_Vector(6);
                VectorXd GFL_Vector(6);
                GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                VectorXd GFR_Vector(6);
                GFR_Vector << 0, -0.05, 0, 0, 0, 0;

                GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.43, 0, 0, 0;

                Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;

                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
            }

            // foot traj generate and walking
            else if (Kubot.CommandFlag == ACT_TEST_WALK || Kubot.CommandFlag == ACT_INF_WALK || Kubot.CommandFlag == ACT_STOP_WALK && Kubot.WALK_READY == true)
            {

                VectorXd GBS_Vector(6);
                VectorXd GFL_Vector(6);
                VectorXd GFR_Vector(6);

                GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.43, 0, 0, Kubot.Base.refpos(5);
                GFL_Vector << Kubot.LFoot.ref_G_pattern_pos(0), Kubot.LFoot.ref_G_pattern_pos(1), Kubot.LFoot.ref_G_pattern_pos(2), 0, 0, Kubot.LFoot.refpos(5);
                GFR_Vector << Kubot.RFoot.ref_G_pattern_pos(0), Kubot.RFoot.ref_G_pattern_pos(1), Kubot.RFoot.ref_G_pattern_pos(2), 0, 0, Kubot.RFoot.refpos(5);

                L_foot_ref_x = GFL_Vector(0);
                L_foot_ref_y = GFL_Vector(1);
                L_foot_ref_z = GFL_Vector(2);

                R_foot_ref_x = GFR_Vector(0);
                R_foot_ref_y = GFR_Vector(1);
                R_foot_ref_z = GFR_Vector(2);

                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);

                L_foot_FKcheck << Kubot.jointToPosition(test_vector, GBS_Vector(0), GBS_Vector(1), GBS_Vector(2));

                L_foot_FK_x = L_foot_FKcheck(0);
                L_foot_FK_y = L_foot_FKcheck(1);
                L_foot_FK_z = L_foot_FKcheck(2);

                R_foot_FK_x = L_foot_FKcheck(3);
                R_foot_FK_y = L_foot_FKcheck(4);
                R_foot_FK_z = L_foot_FKcheck(5);

                // FK 계산출력 식//
                VectorXd zmpFKcheck(3);

                zmpFKcheck << Kubot.ZMPFK(test_vector, L_foot_FKcheck);

                zmpFK_X = zmpFKcheck(0);
                zmpFK_Y = zmpFKcheck(1);
                zmpFK_Z = zmpFKcheck(2);
            }

            if (/*Kubot.ControlMode == CTRLMODE_HOMEPOSE || Kubot.ControlMode == CTRLMODE_STANDUP_TEST || */ Kubot.ControlMode == CTRLMODE_MOTION)
            {

                // if(Kubot.standup_3 == false /* || Kubot.standup_3 == false || Kubot.standup_4 == false*/){

                //     joint[LHR].targetRadian = Kubot.refAngle[1] + Kubot.cosWave(-30*D2R, 1.0 , Kubot.HDLtime, 0);
                //     printf("joint[LHR] compensate \n");

                //     // joint[RHR].targetRadian = Kubot.refAngle[7] + Kubot.cosWave(-30*D2R, 1.0 , Kubot.HDLtime, 0);
                //     // printf("joint[RHR] compensate \n");

                //     joint[LHY].targetRadian = Kubot.refAngle[0];
                //     joint[LHR].targetRadian = Kubot.refAngle[1];
                //     joint[LHP].targetRadian = Kubot.refAngle[2];
                //     joint[LKN].targetRadian = Kubot.refAngle[3];
                //     joint[LAP].targetRadian = Kubot.refAngle[4];
                //     joint[LAR].targetRadian = Kubot.refAngle[5];

                //     joint[RHY].targetRadian = Kubot.refAngle[6];
                //     // joint[RHR].targetRadian = Kubot.refAngle[7];
                //     joint[RHP].targetRadian = Kubot.refAngle[8];
                //     joint[RKN].targetRadian = Kubot.refAngle[9];
                //     joint[RAP].targetRadian = Kubot.refAngle[10];
                //     joint[RAR].targetRadian = Kubot.refAngle[11];

                // }

                // if(Kubot.standup_7 == false /*|| Kubot.standup_7 == false || Kubot.standup_8 == false*/){

                //     joint[RHR].targetRadian = Kubot.refAngle[7] + Kubot.cosWave(30*D2R, 1.0 , Kubot.HDLtime, 0);
                //     printf("joint[RHR] compensate \n");

                //     // joint[LHR].targetRadian = Kubot.refAngle[1] + Kubot.cosWave(15*D2R, 1.0 , Kubot.HDLtime, 0);
                //     // printf("joint[LHR] compensate \n");

                //     joint[LHY].targetRadian = Kubot.refAngle[0];
                //     // joint[LHR].targetRadian = Kubot.refAngle[1];
                //     joint[LHP].targetRadian = Kubot.refAngle[2];
                //     joint[LKN].targetRadian = Kubot.refAngle[3];
                //     joint[LAP].targetRadian = Kubot.refAngle[4];
                //     joint[LAR].targetRadian = Kubot.refAngle[5];

                //     joint[RHY].targetRadian = Kubot.refAngle[6];
                //     joint[RHR].targetRadian = Kubot.refAngle[7];
                //     joint[RHP].targetRadian = Kubot.refAngle[8];
                //     joint[RKN].targetRadian = Kubot.refAngle[9];
                //     joint[RAP].targetRadian = Kubot.refAngle[10];
                //     joint[RAR].targetRadian = Kubot.refAngle[11];

                // }

                joint[LHY].targetRadian = Kubot.refAngle[0];
                joint[LHR].targetRadian = Kubot.refAngle[1];
                joint[LHP].targetRadian = Kubot.refAngle[2];
                joint[LKN].targetRadian = Kubot.refAngle[3];
                joint[LAP].targetRadian = Kubot.refAngle[4];
                joint[LAR].targetRadian = Kubot.refAngle[5];

                joint[RHY].targetRadian = Kubot.refAngle[6];
                joint[RHR].targetRadian = Kubot.refAngle[7];
                joint[RHP].targetRadian = Kubot.refAngle[8];
                joint[RKN].targetRadian = Kubot.refAngle[9];
                joint[RAP].targetRadian = Kubot.refAngle[10];
                joint[RAR].targetRadian = Kubot.refAngle[11];

                // if (Kubot.HDLtime < 1.0 && Kubot.standup_2 == false)
                // {
                //     joint[LHY].targetRadian = Kubot.refAngle[0];
                //     joint[LHR].targetRadian = Kubot.refAngle[1];
                //     joint[LHP].targetRadian = Kubot.refAngle[2];
                //     joint[LKN].targetRadian = Kubot.refAngle[3];
                //     joint[LAP].targetRadian = Kubot.refAngle[4];
                //     joint[LAR].targetRadian = Kubot.refAngle[5];

                //     joint[RHY].targetRadian = Kubot.refAngle[6];
                //     joint[RHR].targetRadian = Kubot.refAngle[7] + (30 * D2R);
                //     joint[RHP].targetRadian = Kubot.refAngle[8];
                //     joint[RKN].targetRadian = Kubot.refAngle[9];
                //     joint[RAP].targetRadian = Kubot.refAngle[10];
                //     joint[RAR].targetRadian = Kubot.refAngle[11];
                // }

                // joint[LHY].targetRadian = Kubot.refAngle[0];
                // joint[LHR].targetRadian = Kubot.refAngle[1];
                // joint[LHP].targetRadian = Kubot.refAngle[2];
                // joint[LKN].targetRadian = Kubot.refAngle[3];
                // joint[LAP].targetRadian = Kubot.refAngle[4];
                // joint[LAR].targetRadian = Kubot.refAngle[5];

                // joint[RHY].targetRadian = Kubot.refAngle[6];
                // joint[RHR].targetRadian = Kubot.refAngle[7];
                // joint[RHP].targetRadian = Kubot.refAngle[8];
                // joint[RKN].targetRadian = Kubot.refAngle[9];
                // joint[RAP].targetRadian = Kubot.refAngle[10];
                // joint[RAR].targetRadian = Kubot.refAngle[11];

                // joint[LSP].targetRadian = Kubot.refAngle[12];     // L_shoulder_pitch_joint 플러스가 어깨 뒤로
                // joint[LER].targetRadian = Kubot.refAngle[13];     // L_elbow_roll_joint 플러스가 좌우로 나란히
                // joint[LHA].targetRadian = Kubot.refAngle[14];     // L_hand_pitch_joint  플러스가 안으로 굽
                // joint[RSP].targetRadian = Kubot.refAngle[15];     // R_shoulder_pitch_joint
                // joint[RER].targetRadian = Kubot.refAngle[16];     // R_elbow_roll_joint
                // joint[RHA].targetRadian = Kubot.refAngle[17];     // R_hand_pitch_joint
                // joint[NYA].targetRadian = Kubot.refAngle[18];     // Neck_yaw_joint
                // joint[HEP].targetRadian = Kubot.refAngle[19];     // head_pitch_joint
                // printf("==== ControlMode check ====\n");
            }

            else if (Kubot.CommandFlag == ACT_INF_WALK || Kubot.CommandFlag == ACT_STOP_WALK)
            {
                joint[LHY].targetRadian = joint_IK[LHY];
                joint[LHR].targetRadian = joint_IK[LHR];
                joint[LHP].targetRadian = joint_IK[LHP];
                joint[LKN].targetRadian = joint_IK[LKN];
                joint[LAP].targetRadian = joint_IK[LAP];
                joint[LAR].targetRadian = joint_IK[LAR];

                joint[RHY].targetRadian = joint_IK[RHY];
                joint[RHR].targetRadian = joint_IK[RHR];
                joint[RHP].targetRadian = joint_IK[RHP];
                joint[RKN].targetRadian = joint_IK[RKN];
                joint[RAP].targetRadian = joint_IK[RAP];
                joint[RAR].targetRadian = joint_IK[RAR];

                // std::cout << "joint_IK:\n" << joint_IK << std::endl;
                // joint[LSP].targetRadian = Kubot.refAngle[12];     // L_shoulder_pitch_joint 플러스가 어깨 뒤로
                // joint[LER].targetRadian = Kubot.refAngle[13];     // L_elbow_roll_joint 플러스가 좌우로 나란히
                // joint[LHA].targetRadian = Kubot.refAngle[14];     // L_hand_pitch_joint  플러스가 안으로 굽
                // joint[RSP].targetRadian = Kubot.refAngle[15];     // R_shoulder_pitch_joint
                // joint[RER].targetRadian = Kubot.refAngle[16];     // R_elbow_roll_joint
                // joint[RHA].targetRadian = Kubot.refAngle[17];     // R_hand_pitch_joint
                // joint[NYA].targetRadian = Kubot.refAngle[18];     // Neck_yaw_joint
                // joint[HEP].targetRadian = Kubot.refAngle[19];     // head_pitch_joint
                // printf("===========================");
            }

            // joint[LHY].targetRadian = joint_IK[LHY];
            // joint[LHR].targetRadian = joint_IK[LHR];
            // joint[LHP].targetRadian = joint_IK[LHP];
            // joint[LKN].targetRadian = joint_IK[LKN];
            // joint[LAP].targetRadian = joint_IK[LAP];
            // joint[LAR].targetRadian = joint_IK[LAR];

            // joint[RHY].targetRadian = joint_IK[RHY];
            // joint[RHR].targetRadian = joint_IK[RHR];
            // joint[RHP].targetRadian = joint_IK[RHP];
            // joint[RKN].targetRadian = joint_IK[RKN];
            // joint[RAP].targetRadian = joint_IK[RAP];
            // joint[RAR].targetRadian = joint_IK[RAR];

            // std::cout << "joint_IK:\n" << joint_IK << std::endl;

            // if(Kubot.ControlMode != CTRLMODE_HOMEPOSE && Kubot.WALK_READY == true){
            //     printf("\n Compensate START \n");
            //     // Compensate();
            //     // std::cout << "\n Kubot.joint[LHR].compensate_refAngle : \n" <<  Kubot.joint[LHR].compensate_refAngle << std::endl;
            //     // std::cout << "\n joint[LHR].targetRadian: \n" << joint_IK[LHR] + Kubot.joint[LHR].compensate_refAngle << std::endl;

            //     // joint[LHY].targetRadian = joint_IK[LHY];
            //     // printf("\n 1111111111 START \n");
            //     joint[LHR].targetRadian = joint_IK[LHR] + Kubot.joint[LHR].compensate_refAngle;  // 여기서 죽는 듯
            //     // printf("jonit[LHR].targetRadian : %f \n", joint[LHR].targetRadian);
            //     // printf("\n 22222222222222 START \n");
            //     // joint[LHP].targetRadian = joint_IK[LHP];
            //     // printf("\n 33333333333 START \n");
            //     // joint[LKN].targetRadian = joint_IK[LKN];
            //     // printf("\n 444444444444 START \n");
            //     // joint[LAP].targetRadian = joint_IK[LAP];
            //     // printf("\n 55555555555555555 START \n");
            //     // joint[LAR].targetRadian = joint_IK[LAR];
            //     // printf("\n 666666666666 START \n");

            //     // joint[RHY].targetRadian = joint_IK[RHY];
            //     // printf("\n 77777777777777777 START \n");
            //     // joint[RHR].targetRadian = joint_IK[RHR] + Kubot.joint[RHR].compensate_refAngle;
            //     // printf("jonit[RHR].targetRadian : %f \n", joint[RHR].targetRadian);
            //     // printf("\n 888888888 START \n");
            //     // joint[RHP].targetRadian = joint_IK[RHP];
            //     // printf("\n 9999999999999 START \n");
            //     // joint[RKN].targetRadian = joint_IK[RKN];
            //     // printf("\n 101010101010100101 START \n");
            //     // joint[RAP].targetRadian = joint_IK[RAP];
            //     // printf("\n 12121212121212121212 START \n");
            //     // joint[RAR].targetRadian = joint_IK[RAR];
            //     // printf("\n 13131313131311313 START \n");

            //     // printf("jonit[LHR].targetRadian : %f \n", joint[LHR].targetRadian);
            //     // printf("jonit[RHR].targetRadian : %f \n", joint[RHR].targetRadian);

            //     printf("\n Compensate END \n");

            // }
            // else {
            //     joint[LHY].targetRadian = joint_IK[LHY];
            //     joint[LHR].targetRadian = joint_IK[LHR];
            //     joint[LHP].targetRadian = joint_IK[LHP];
            //     joint[LKN].targetRadian = joint_IK[LKN];
            //     joint[LAP].targetRadian = joint_IK[LAP];
            //     joint[LAR].targetRadian = joint_IK[LAR];

            //     joint[RHY].targetRadian = joint_IK[RHY];
            //     joint[RHR].targetRadian = joint_IK[RHR];
            //     joint[RHP].targetRadian = joint_IK[RHP];
            //     joint[RKN].targetRadian = joint_IK[RKN];
            //     joint[RAP].targetRadian = joint_IK[RAP];
            //     joint[RAR].targetRadian = joint_IK[RAR];

            // }
        }
    }

    jointcontroller();

    RosTopicPublish();

    if (Kubot.Move_current == true)
    {
        int timeCnt;
        timeCnt = (int)((Kubot.realTime - Kubot.startTime) * 1000 + 0.001); //[msec]

        Kubot.save_jointData[0][timeCnt] = timeCnt;
        for (int i = 2; i <= 13; i++)
        {
            Kubot.save_jointData[i - 1][timeCnt] = joint[i - 2].targetRadian;
        }
        for (int i = 14; i <= 25; i++)
        {
            Kubot.save_jointData[i - 1][timeCnt] = joint[i - 14].actualRadian;
        }

        Kubot.save_previewData[0][timeCnt] = timeCnt;
        Kubot.save_previewData[1][timeCnt] = Kubot.zmp.previewControl.X.m_ref;   // ZMP_ref_X
        Kubot.save_previewData[2][timeCnt] = Kubot.zmp.previewControl.Y.m_ref;   // ZMP_ref_Y
        Kubot.save_previewData[3][timeCnt] = Kubot.zmp.previewControl.X.old_zmp; // ZMP_calculate_X
        Kubot.save_previewData[4][timeCnt] = Kubot.zmp.previewControl.Y.old_zmp; // ZMP_calculate_Y
        Kubot.save_previewData[5][timeCnt] = Kubot.zmp.previewControl.X.CoM;     // Com_X
        Kubot.save_previewData[6][timeCnt] = Kubot.zmp.previewControl.Y.CoM;     // Com_Y
        Kubot.save_previewData[7][timeCnt] = 0.43;                               // Com_Z
        Kubot.save_previewData[8][timeCnt] = zmpFK_X;                            // FK_X
        Kubot.save_previewData[9][timeCnt] = zmpFK_Y;                            // FK_Y
        Kubot.save_previewData[10][timeCnt] = zmpFK_Z;                           // FK_Z

        Kubot.save_footData[0][timeCnt] = timeCnt;
        Kubot.save_footData[1][timeCnt] = L_foot_ref_x; // L_ref_X
        Kubot.save_footData[2][timeCnt] = L_foot_ref_y; // L_ref_Y
        Kubot.save_footData[3][timeCnt] = L_foot_ref_z; // L_ref_Z
        Kubot.save_footData[4][timeCnt] = R_foot_ref_x; // R_ref_X
        Kubot.save_footData[5][timeCnt] = R_foot_ref_y; // R_ref_Y
        Kubot.save_footData[6][timeCnt] = R_foot_ref_z; // R_ref_Z
        Kubot.save_footData[7][timeCnt] = L_foot_FK_x;  // L_FK_X
        Kubot.save_footData[8][timeCnt] = L_foot_FK_y;  // L_FK_Y
        Kubot.save_footData[9][timeCnt] = L_foot_FK_z;  // L_FK_Z
        Kubot.save_footData[10][timeCnt] = R_foot_FK_x; // R_FK_X
        Kubot.save_footData[11][timeCnt] = R_foot_FK_y; // R_FK_Y
        Kubot.save_footData[12][timeCnt] = R_foot_FK_z; // R_FK_Z

        // printf("data save // time : %d \n ",timeCnt);

        KubotModesp_pub.publish(KubotModesp_msg);
    }
}

void gazebo::src23_plugin::setjoints() // plugin에다가 joints name 설정 .sdf파일에서 설정한 이름이랑 확인하기
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("LP");
    L_Hip_roll_joint = this->model->GetJoint("LPm");
    L_Hip_pitch_joint = this->model->GetJoint("LPd");
    L_Knee_joint = this->model->GetJoint("LK");
    L_Ankle_pitch_joint = this->model->GetJoint("LA");
    L_Ankle_roll_joint = this->model->GetJoint("LF");

    R_Hip_yaw_joint = this->model->GetJoint("RP");
    R_Hip_roll_joint = this->model->GetJoint("RPm");
    R_Hip_pitch_joint = this->model->GetJoint("RPd");
    R_Knee_joint = this->model->GetJoint("RK");
    R_Ankle_pitch_joint = this->model->GetJoint("RA");
    R_Ankle_roll_joint = this->model->GetJoint("RF");
}

void gazebo::src23_plugin::getjointdata()
{
    /*
     * Get encoder and velocity data of each joint[j].targetRadian = joint_h[j];
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);
    // printf("before for \n");

    for (int j = 0; j < 12; j++)
    { // ndof -> 12
        joint[j].actualDegree = joint[j].actualRadian * R2D;

        test_vector(j) = joint[j].actualRadian;
    }

    // printf("complete for\n");
    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    for (int j = 0; j < nDoF; j++)
    {

        // printf(C_BLUE "joint[%d].targetVelocity = %f\n" C_RESET, j, joint[j].targetVelocity);
    }

    joint[LHY].actualTorque = L_Hip_yaw_joint->GetForce(0);
    joint[LHR].actualTorque = L_Hip_roll_joint->GetForce(0);
    joint[LHP].actualTorque = L_Hip_pitch_joint->GetForce(0);
    joint[LKN].actualTorque = L_Knee_joint->GetForce(0);
    joint[LAP].actualTorque = L_Ankle_pitch_joint->GetForce(0);
    joint[LAR].actualTorque = L_Ankle_roll_joint->GetForce(0);

    joint[RHY].actualTorque = R_Hip_yaw_joint->GetForce(0);
    joint[RHR].actualTorque = R_Hip_roll_joint->GetForce(0);
    joint[RHP].actualTorque = R_Hip_pitch_joint->GetForce(0);
    joint[RKN].actualTorque = R_Knee_joint->GetForce(0);
    joint[RAP].actualTorque = R_Ankle_pitch_joint->GetForce(0);
    joint[RAR].actualTorque = R_Ankle_roll_joint->GetForce(0);

    for (int j = 0; j < nDoF; j++)
    {

        joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    }
}

void gazebo::src23_plugin::setsensor() // sdf파일에 있는 sensor를 설정하는 함수
{
    Sensor = sensors::get_sensor("IMU");
    IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);
}

void gazebo::src23_plugin::getsensordata()
{
    // IMU sensor data
    double IMUdata[3];

    IMUdata[0] = IMU->Orientation().Euler()[0];
    IMUdata[1] = IMU->Orientation().Euler()[1];
    IMUdata[2] = IMU->Orientation().Euler()[2];

    // printf(C_BLUE "IMU roll = %f\n" C_RESET, IMUdata[0]*R2D );
    // printf(C_RED "IMU pitch = %f\n" C_RESET, IMUdata[1]*R2D );
    // printf(C_YELLOW "IMU yaw = %f\n" C_RESET, IMUdata[2]*R2D );

    // IMU_dtheta[Roll] = IMU->AngularVelocity(false)[Roll];
    // IMU_dtheta[Pitch] = IMU->AngularVelocity(false)[Pitch];
    // IMU_dtheta[Yaw] = IMU->AngularVelocity(false)[Yaw];
}

void gazebo::src23_plugin::jointcontroller()
{
    for (int j = 0; j < nDoF; j++)
    {
        joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));
    }

    //* Update target torque in gazebo simulation
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);
}

void gazebo::src23_plugin::initializejoint()
{
    /*
     * Initialize joint variables for joint control
     */
    joint[0].init_targetradian = (0 * D2R);    // L_Hip_yaw_joint
    joint[1].init_targetradian = (0 * D2R);    // L_Hip_roll_joint
    joint[2].init_targetradian = (-45 * D2R);  // L_Hip_pitch_joint
    joint[3].init_targetradian = (90 * D2R);   // L_Knee_joint
    joint[4].init_targetradian = (-45 * D2R);  // L_Ankle_pitch_joint
    joint[5].init_targetradian = (0 * D2R);    // L_Ankle_roll_joint
    joint[6].init_targetradian = (0 * D2R);    // R_Hip_yaw_joint
    joint[7].init_targetradian = (0 * D2R);    // R_Hip_roll_joint
    joint[8].init_targetradian = (-45 * D2R);  // R_Hip_pitch_joint
    joint[9].init_targetradian = (90 * D2R);   // R_Knee_joint
    joint[10].init_targetradian = (-45 * D2R); // R_Ankle_pitch_joint
    joint[11].init_targetradian = (0 * D2R);   // R_Ankle_roll_joint
}

void gazebo::src23_plugin::setjointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 200; // 100
    joint[LHR].Kp = 170; // 110
    joint[LHP].Kp = 210; // 130
    joint[LKN].Kp = 230; // 200
    joint[LAP].Kp = 150; // 100
    joint[LAR].Kp = 150; // 100

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[LHY].Kd = 0.08; // 0.03
    joint[LHR].Kd = 0.09; // 0.05
    joint[LHP].Kd = 0.10; // 0.08
    joint[LKN].Kd = 0.11; // 0.07
    joint[LAP].Kd = 0.06; // 0.03
    joint[LAR].Kd = 0.06; // 0.03

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;
}

void gazebo::src23_plugin::InitRosTopic()
{
    // ROS Publishers
    KubotModesp_pub = nh.advertise<std_msgs::Float64>("command/KubotMode", 1000);

    // base
    P_preview_COM_x = nh.advertise<std_msgs::Float64>("COM_x", 1000);
    P_preview_ref_ZMP_x = nh.advertise<std_msgs::Float64>("zmp_ref_x", 1000);
    P_preview_COM_y = nh.advertise<std_msgs::Float64>("COM_y", 1000);
    P_preview_ref_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_ref_y", 1000);
    P_ZMP_x = nh.advertise<std_msgs::Float64>("zmp_X", 1000);
    P_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_Y", 1000);

    P_preview_FK_x = nh.advertise<std_msgs::Float64>("zmpFK_X", 1000);
    P_preview_FK_y = nh.advertise<std_msgs::Float64>("zmpFK_Y", 1000);

    // L_foot
    P_L_foot_ref_x = nh.advertise<std_msgs::Float64>("L_foot_ref_X", 1000);
    P_L_foot_ref_z = nh.advertise<std_msgs::Float64>("L_foot_ref_Z", 1000);
    P_L_foot_FK_x = nh.advertise<std_msgs::Float64>("L_foot_FK_X", 1000);
    P_L_foot_FK_z = nh.advertise<std_msgs::Float64>("L_foot_FK_Z", 1000);

    // R_foot
    P_R_foot_ref_x = nh.advertise<std_msgs::Float64>("R_foot_ref_X", 1000);
    P_R_foot_ref_z = nh.advertise<std_msgs::Float64>("R_foot_ref_Z", 1000);
    P_R_foot_FK_x = nh.advertise<std_msgs::Float64>("R_foot_FK_X", 1000);
    P_R_foot_FK_z = nh.advertise<std_msgs::Float64>("R_foot_FK_Z", 1000);

    //////////////////////////////////
    P_time_sec = nh.advertise<std_msgs::Float64>("time_sec", 1000);
    P_time_old_sec = nh.advertise<std_msgs::Float64>("time_old_sec", 1000);
    P_time_init_sec = nh.advertise<std_msgs::Float64>("time_init_sec", 1000);
    P_time_wsec = nh.advertise<std_msgs::Float64>("time_wsec", 1000);
    // P_predicted_time = nh.advertise<std_msgs::Float64>("predicted_time", 1000);
    // P_predicted_step = nh.advertise<std_msgs::Float64>("predicted_step", 1000);

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_topic", 1000);
}

void gazebo::src23_plugin::RosTopicPublish()
{
    //////////////base//////////////
    m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
    m_preview_ref_ZMP_y.data = Kubot.zmp.previewControl.Y.m_ref;
    m_preview_COM_x.data = Kubot.zmp.previewControl.X.CoM;
    m_preview_ref_ZMP_x.data = Kubot.zmp.previewControl.X.m_ref;
    m_ZMP_x.data = Kubot.zmp.previewControl.X.old_zmp;
    m_ZMP_y.data = Kubot.zmp.previewControl.Y.old_zmp;

    m_preview_FK_x.data = zmpFK_X;
    m_preview_FK_y.data = zmpFK_Y;

    //////////////L_foot//////////////
    m_L_foot_ref_x.data = L_foot_ref_x;
    m_L_foot_ref_z.data = L_foot_ref_z;
    m_L_foot_FK_x.data = L_foot_FK_x;
    m_L_foot_FK_z.data = L_foot_FK_z;

    //////////////R_foot//////////////
    m_R_foot_ref_x.data = R_foot_ref_x;
    m_R_foot_ref_z.data = R_foot_ref_z;
    m_R_foot_FK_x.data = R_foot_FK_x;
    m_R_foot_FK_z.data = R_foot_FK_z;

    //////////////////
    P_preview_COM_y.publish(m_preview_COM_y);
    P_preview_ref_ZMP_y.publish(m_preview_ref_ZMP_y);
    P_preview_COM_x.publish(m_preview_COM_x);
    P_preview_ref_ZMP_x.publish(m_preview_ref_ZMP_x);
    P_ZMP_x.publish(m_ZMP_x);
    P_ZMP_y.publish(m_ZMP_y);
    P_preview_FK_x.publish(m_preview_FK_x);
    P_preview_FK_y.publish(m_preview_FK_y);

    //////////////L_foot//////////////
    P_L_foot_ref_x.publish(m_L_foot_ref_x);
    P_L_foot_ref_z.publish(m_L_foot_ref_z);
    P_L_foot_FK_x.publish(m_L_foot_FK_x);
    P_L_foot_FK_z.publish(m_L_foot_FK_z);

    //////////////R_foot//////////////
    P_R_foot_ref_x.publish(m_R_foot_ref_x);
    P_R_foot_ref_z.publish(m_R_foot_ref_z);
    P_R_foot_FK_x.publish(m_R_foot_FK_x);
    P_R_foot_FK_z.publish(m_R_foot_FK_z);

    /////////////////////////////time
    m_time_sec.data = Kubot.walking.time.sec;
    m_time_old_sec.data = Kubot.walking.time.old_sec;
    m_time_init_sec.data = Kubot.walking.time.init_sec;
    m_time_wsec.data = Kubot.walking.time.wsec;
    // m_predicted_time.data = Kubot.predicted_walking_time;
    // m_predicted_step.data = Kubot.predicted_walking_step_num;

    P_time_sec.publish(m_time_sec);
    P_time_old_sec.publish(m_time_old_sec);
    P_time_init_sec.publish(m_time_init_sec);
    P_time_wsec.publish(m_time_wsec);
    // P_predicted_time.publish(m_predicted_time);
    // P_predicted_step.publish(m_predicted_step);

    cmd_vel_pub_.publish(cmd_vel_msg);
}

// void gazebo::src23_plugin::Compensate() {
//     VectorXd joint_IK(12);

//     joint[LHY].targetRadian = joint_IK[LHY];
//     joint[LHR].targetRadian = joint_IK[LHR] + Kubot.joint[LHR].compensate_refAngle;
//     joint[LHP].targetRadian = joint_IK[LHP];
//     joint[LKN].targetRadian = joint_IK[LKN];
//     joint[LAP].targetRadian = joint_IK[LAP];
//     joint[LAR].targetRadian = joint_IK[LAR];

//     joint[RHY].targetRadian = joint_IK[RHY];
//     joint[RHR].targetRadian = joint_IK[RHR] + Kubot.joint[RHR].compensate_refAngle;
//     joint[RHP].targetRadian = joint_IK[RHP];
//     joint[RKN].targetRadian = joint_IK[RKN];
//     joint[RAP].targetRadian = joint_IK[RAP];
//     joint[RAR].targetRadian = joint_IK[RAR];

//     printf("jonit[LHR].targetRadian : %f \n", joint[LHR].targetRadian);
//     printf("jonit[RHR].targetRadian : %f \n", joint[RHR].targetRadian);

// }