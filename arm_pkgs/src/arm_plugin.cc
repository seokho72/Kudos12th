#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>
#include <time.h>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>

#include <gazebo/common/common.hh> 
#include <gazebo/common/Plugin.hh> //model plugin gazebo API에서 확인 
#include <gazebo/physics/physics.hh> //우리는 ODE physics 사용 
#include <gazebo/sensors/sensors.hh> //IMU sensor 사용 

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> //topic을 어떤 형태로 보내는지에 따라 사용되는 헤더파일이다.

#include <functional>
#include <ignition/math/Vector3.hh>

// 1
#include "Kudos.h" 

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI 

//* Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

using namespace std;
//여기까지가 책으로 따지자면 목차이다. namespace까지

namespace gazebo {

    class arm_plugin : public ModelPlugin
    {
        //*** Variables for Kubot Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        physics::ModelPtr model; // model = _model 관련

        physics::JointPtr Link1_joint;
        physics::JointPtr Link2_joint;

        //* Index setting for each joint
        // physics::JointPtr L_Foot_joint;
        // physics::JointPtr R_Foot_joint;


// 2, 5줄
        //* ROS Subscribe
        ros::NodeHandle nh;     // ros 통신을 위한 node handle 선언 
        ros::Subscriber KudosModesp;  // ROS subscriber 선언
        int ControlMode_by_ROS = 1;   

        //* ROS Publish
        ros::Publisher KudosModesp_pub;  // ROS pubscriber 선언
        std_msgs::Float64 KudosModesp_msg; // 메시지 형식

        enum
        { //wst는 torso joint 같음
            LK1 = 0, LK2
        };



        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot
//구조체는 변수 집합이라고 생각하면 편하다.
        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]
            double init_targetradian;

            double targetRadian_interpolation; //The interpolated target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;
        } ROBO_JOINT;
        ROBO_JOINT* joint;                

        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU; //imu 센서 추가

         // 3, 한줄
        Kudos kudos; // Kudos이라는 이름을 가진 kudos class 선언함과 동시에 변수들을 다 갖고옴


    public :
            //*** Functions for Kubot Simulation in Gazebo ***//
          void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
          void UpdateAlgorithm(); // Algorithm update while simulation
          
          void setjoints(); // Get each joint data from [physics::ModelPtr _model]    
          void getjointdata(); // Get encoder data of each joint

          void jointcontroller();
          
          void initializejoint(); 
          void setjointPIDgain(); 

            // 4, 한줄
          void KudosMode(const std_msgs::Int32 &msg);

    };
    GZ_REGISTER_MODEL_PLUGIN(arm_plugin);
    
}

// 5, 전부
void gazebo::arm_plugin::KudosMode(const std_msgs::Int32 &msg)
{
    // ROS_INFO("I heard: [%d]",msg.data);

    ControlMode_by_ROS = msg.data;
    
    switch (ControlMode_by_ROS) {

    case 1:
        kudos.ControlMode = WRITEMODE;
        kudos.CommandFlag = NO1;
        printf("Kudos No 1! \n");
        
        kudos.Move_current = true;
        ControlMode_by_ROS = 0;
        break;
    
    case 2:
        kudos.ControlMode = WRITEMODE;
        kudos.CommandFlag = RETURN_POSE;
        printf("RETURN POSE! \n");
        
        kudos.Move_current = true;
        ControlMode_by_ROS = 0;
        break;

    default:
         break;
    }

}

void gazebo::arm_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{

    // 6, 세줄
    kudos.Move_current = false;
    printf("\nLoad_do\n");
    KudosModesp = nh.subscribe("/KudosMode",1,&gazebo::arm_plugin::KudosMode,this);
  
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    setjoints();
    //sdf변환하기 전에 urdf파일에서 조인트가 몇개인지 인식을 함.
    //인식이 끝나면 joint 갯수에 맞게 구조체를 여러개 생성
    //Robotjoint 구조체를 joint 갯수만큼 복제되는 거임

    nDoF = 2; // Get degrees of freedom, except position and orientation of the robot
    //우리가 만든 joint는 12개로 지정
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct    
    
    initializejoint();
    setjointPIDgain();
    //* setting for getting dt

    // 7, 한줄
    KudosModesp_pub = nh.advertise<std_msgs::Float64>("command/KudosMode", 1000);

    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&arm_plugin::UpdateAlgorithm, this));
    
}

void gazebo::arm_plugin::UpdateAlgorithm()
{
    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //        cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;
    
    getjointdata();
    // printf(C_BLUE "time = %f\n" C_RESET, time);

    // 8
    static int con_count = 0;

    //* Real or simulated real-time thread time setting
    if (con_count % (int) (tasktime * 1000 + 0.001) == 0)
    {

        //* ControlMode
        switch (kudos.ControlMode) 
        {

        case WRITEMODE:

            break;

        }

        //* CommandFlag
        switch (kudos.CommandFlag) 
        {

        case NO1:
            // WalkReady : Preparing the robot for walking
            if (kudos.Move_current == true) {
                kudos.NO1(2.0);
                 for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = kudos.refAngle[j];
                }
            }
            else {
                kudos.CommandFlag = NONE_ACT;
                printf("NO1 !\n");

            }
            break;

        case RETURN_POSE:

            if (kudos.Move_current == true) {
                kudos.ReturnPose(2.0);
                 for (int j = 0; j < nDoF; j++) {
                     joint[j].targetRadian = kudos.refAngle[j];
                }
            }
            else {
                printf("RETURN_POSE COMPLETE !\n");
                kudos.CommandFlag = NONE_ACT;
            }
            break;

        
        case NONE_ACT:

            break;
        }

    }    
// 여기까지

    jointcontroller();        

     // 9, 한 줄
    KudosModesp_pub.publish(KudosModesp_msg); 
}

void gazebo::arm_plugin::setjoints() //plugin에다가 joints name 설정 .sdf파일에서 설정한 이름이랑 확인하기
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    Link1_joint = this->model->GetJoint("Link1_joint");
    Link2_joint = this->model->GetJoint("Link2_joint");
}

void gazebo::arm_plugin::getjointdata()
{
    /*
     * Get encoder and velocity data of each joint[j].targetRadian = joint_h[j];
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    
// 10, 기존꺼 지우고 6줄
    joint[LK1].actualRadian = Link1_joint->Position(0);
    joint[LK2].actualRadian = Link2_joint->Position(0);

    joint[LK1].actualVelocity = Link1_joint->GetVelocity(0);
    joint[LK2].actualVelocity = Link2_joint->GetVelocity(0);

    joint[LK1].actualTorque = Link1_joint->GetForce(0);
    joint[LK2].actualTorque = Link2_joint->GetForce(0);
    
   
    }


void gazebo::arm_plugin::jointcontroller()
{
        static double pre_rad[2];

        
           // 11,for문
        for (int j = 0; j < nDoF; j++){           

            joint[j].targetTorque = joint[j].Kp*(joint[j].targetRadian-joint[j].actualRadian) \
                                  + joint[j].Kd*(joint[j].targetVelocity -joint[j].actualVelocity);

        }

        //* Update target torque in gazebo simulation
        Link1_joint->SetForce(0, joint[LK1].targetTorque);
        Link2_joint->SetForce(0, joint[LK2].targetTorque);

        
}

void gazebo::arm_plugin::initializejoint()
{
    /*
     * Initialize joint variables for joint control
     */
        joint[0].init_targetradian= 45*D2R; // Link1_joint
        joint[1].init_targetradian= 60*D2R; // Link2_joint

}

void gazebo::arm_plugin::setjointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
        joint[LK1].Kp = 1;
        joint[LK2].Kp = 1;

        joint[LK1].Kd = 0.05;
        joint[LK2].Kd = 0.05;

}

