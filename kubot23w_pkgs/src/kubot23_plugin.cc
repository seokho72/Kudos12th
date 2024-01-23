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
#include <gazebo/common/Plugin.hh> //model plugin gazebo API에서 확인 
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


#include <kubot23w_pkgs/Kubot_control_msgs.h>

// KUDOS CKubot Header 파일 선언
#include "kudos/CKubot.h"
// #include "kudos/kinematics.h"


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
//여기까지가 책으로 따지자면 목차. namespace까지 

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
//Eigen 파일 안에 있는 namespace를 사용하겠다.

VectorXd test_vector(20);



namespace gazebo {

    class kubot23_plugin : public ModelPlugin
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

        physics::JointPtr L_shoulder_pitch_joint;
        physics::JointPtr L_elbow_roll_joint;
        physics::JointPtr L_hand_pitch_joint;

        physics::JointPtr R_shoulder_pitch_joint;
        physics::JointPtr R_elbow_roll_joint;
        physics::JointPtr R_hand_pitch_joint;

        physics::JointPtr Neck_yaw_joint;
        physics::JointPtr head_pitch_joint;
        //* Index setting for each joint
        // physics::JointPtr L_Foot_joint;
        // physics::JointPtr R_Foot_joint;


        //* ROS Subscribe
        ros::NodeHandle nh; // ros 통신을 위한 node handle 선언 
        ros::Subscriber KubotModesp;
        int ControlMode_by_ROS = 1;

        //FOOTSTEP
        int StartFootCheck;
        bool FSW_STOP;
        
        //ZMP
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
        
        //Preview
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
        std_msgs::Float64 m_preview_FK_y;    /// 박사님 과제 FK

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
        ros::Publisher P_preview_FK_y;       /// 박사님 과제 FK

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

        std_msgs::Float64 KubotModesp_msg;     
        // std_msgs::Float64 LHY_msg;
        // std_msgs::Float64 LHR_msg;
        // std_msgs::Float64 LHP_msg;
        // std_msgs::Float64 LKN_msg;
        // std_msgs::Float64 LAP_msg;
        // std_msgs::Float64 LAR_msg;

        ros::Subscriber Kubot_control_mode_sub;
        ros::Subscriber Kubot_joy_sub;

        enum
        { //wst는 torso joint 같음
            LHY = 0, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR, LSP, LER, LHA, RSP, RER, RHA, NYA, HEP
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot
//구조체는 변수 집합이라고 생각하면 편하다.

        // int Q0 = Kubot.Q0;
        // int Qi = Kubot.Qi;
        // int Qj = Kubot.Qj;
        // int Qk = Kubot.Qk;


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

        bool joint_by_algorithm_update = false;
         

        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU; //imu 센서 추가

        CKubot Kubot; // Kubot이라는 이름을 가진 CKubot class 선언함과 동시에 변수들을 다 갖고옴


        //* ROS Subscribe
        // ros::NodeHandle nh;
        // ros::Subscriber RoS_Mode;
        // ros::Subscriber Save_Mode;
        // int ControlMode_by_ROS = 0;
        // bool Balancing_Control_Mode_by_ROS = 0;

    public :
            //*** Functions for Kubot Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointUpdateByAlgorithm(); // joint ref update by control algorithm
        
        void setjoints(); // Get each joint data from [physics::ModelPtr _model]    
        void getjointdata(); // Get encoder data of each joint
        
        void setsensor();
        void getsensordata();

        void jointcontroller();


        void initializejoint(); // 추가한
        void setjointPIDgain(); // 함수

        //*ROS subscribe
        // void ros_MODE(const std_msgs::Int32 &msg);          
        void KubotMode(const std_msgs::Int32 &msg);
        void Kubot_control_callback(const kubot23w_pkgs::Kubot_control_msgs &msg);
        void Kubot_joy_callback(const sensor_msgs::Joy &joy_msgs);
        

    };
    GZ_REGISTER_MODEL_PLUGIN(kubot23_plugin);
    
}



//model.sdf파일에 대한 정보를 불러오는 함수
void gazebo::kubot23_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //Kubot.ControlMode = CTRLMODE_WALKREADY; // wave 함수 초기

    // Kubot.Move_current = false;
    printf("\nLoad_do\n");
    KubotModesp = nh.subscribe("/KubotMode",1,&gazebo::kubot23_plugin::KubotMode,this);
    //0626
    Kubot_control_mode_sub = nh.subscribe("/Kubot_Control_Msg",1,&gazebo::kubot23_plugin::Kubot_control_callback,this);
    Kubot_joy_sub = nh.subscribe("/joy",1,&gazebo::kubot23_plugin::Kubot_joy_callback,this);

    
    //* ROS Subscribe
    // Save_Mode = nh.subscribe("save_step", 1, &gazebo::kubot23_plugin::data_save_Execution, this);
    // RoS_Mode = nh.subscribe("rok_mode", 1, &gazebo::kubot23_plugin::ros_MODE, this);
    
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    setjoints();
    //sdf변환하기 전에 urdf파일에서 조인트가 몇개인지 인식을 함.
    //인식이 끝나면 joint 갯수에 맞게 구조체를 여러개 생성
    //Robotjoint 구조체를 joint 갯수만큼 복제되는 거임
    setsensor();


    nDoF = 20; // Get degrees of freedom, except position and orientation of the robot
    //우리가 만든 joint는 12개로 지정
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct    
    
    initializejoint();
    setjointPIDgain();


    Kubot.initializeBkubot();
    
    Kubot.setWalkingReadyPos(0,0,0.38);


    //ROS Publishers
    KubotModesp_pub = nh.advertise<std_msgs::Float64>("command/KubotMode", 1000);

    //base
    P_preview_COM_x = nh.advertise<std_msgs::Float64>("COM_x", 1000);
    P_preview_ref_ZMP_x = nh.advertise<std_msgs::Float64>("zmp_ref_x", 1000);
    P_preview_COM_y = nh.advertise<std_msgs::Float64>("COM_y", 1000);
    P_preview_ref_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_ref_y", 1000);
    P_ZMP_x = nh.advertise<std_msgs::Float64>("zmp_X", 1000);
    P_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_Y", 1000);

    P_preview_FK_x = nh.advertise<std_msgs::Float64>("zmpFK_X", 1000);
    P_preview_FK_y = nh.advertise<std_msgs::Float64>("zmpFK_Y", 1000);

    //L_foot
    P_L_foot_ref_x = nh.advertise<std_msgs::Float64>("L_foot_ref_X", 1000);
    P_L_foot_ref_z = nh.advertise<std_msgs::Float64>("L_foot_ref_Z", 1000);
    P_L_foot_FK_x = nh.advertise<std_msgs::Float64>("L_foot_FK_X", 1000);
    P_L_foot_FK_z = nh.advertise<std_msgs::Float64>("L_foot_FK_Z", 1000);

    //R_foot
    P_R_foot_ref_x = nh.advertise<std_msgs::Float64>("R_foot_ref_X", 1000);
    P_R_foot_ref_z = nh.advertise<std_msgs::Float64>("R_foot_ref_Z", 1000);
    P_R_foot_FK_x = nh.advertise<std_msgs::Float64>("R_foot_FK_X", 1000);
    P_R_foot_FK_z = nh.advertise<std_msgs::Float64>("R_foot_FK_Z", 1000);



           
    // LHY_pub = nh.advertise<std_msgs::Float64>("command_joint/LHY", 1000);//왼발 x좌표 ref값
    // LHR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);//왼발 y좌표 ref값
    // LHP_pub = nh.advertise<std_msgs::Float64>("command_joint/LHP", 1000);
    // LKN_pub = nh.advertise<std_msgs::Float64>("command_joint/LKN", 1000);
    // LAP_pub = nh.advertise<std_msgs::Float64>("command_joint/LAP", 1000);
    // LAR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);
    //* setting for getting dt

    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&kubot23_plugin::UpdateAlgorithm, this));

}
void gazebo::kubot23_plugin::Kubot_joy_callback(const sensor_msgs::Joy &joy_msgs){
    printf("stamp : %d\n",joy_msgs.header.seq);
    printf("joy_data[0] :%f\n",joy_msgs.axes[0]);
    printf("joy_data[1] :%f\n",joy_msgs.axes[1]);
    printf("joy_data[2] :%f\n",joy_msgs.axes[2]);
    if(Kubot.joy_flag == true){ // sony_joystick
    if(joy_msgs.buttons[2] == 1){ // 세모 : 14번
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
    if(joy_msgs.buttons[1] == 1){ // O : 2번
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");
    }
    if(joy_msgs.buttons[0] == 1){ // O : STOP
       if(Kubot.stop_msgs == 0){
       Kubot.stop_msgs = 1;
       printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");
       }
    }
    if(Kubot.Move_current == true){
    Kubot.FBSize_msgs = joy_msgs.axes[1]*0.05;
    Kubot.LRSize_msgs = joy_msgs.axes[3]*0.025;
    Kubot.TurnSize_msgs = joy_msgs.axes[0]*0.15;
    }
    }

    else{ // RcLab.joystick
/////////////////////////////////////////////////////////
    if(joy_msgs.axes[5] == 1){ // O : 2번
        if(Kubot.joy_cnt < 5 && Kubot.joy_cnt >= 0){
            Kubot.joy_cnt ++;
            Kubot.joy_FB_ref = 0.02 * Kubot.joy_cnt;
                printf("\n\n\n\n\n\n\n==================");
                printf("Kubot.joy_cnt : %d",Kubot.joy_cnt);
                printf("Kubot.joy_FB_ref : %lf",Kubot.joy_FB_ref);
        }
    }
    else if(joy_msgs.axes[5] == -1){
        if(Kubot.joy_cnt <= 5  && Kubot.joy_cnt > 0){
            Kubot.joy_cnt --;
            Kubot.joy_FB_ref = 0.02 * Kubot.joy_cnt;
            printf("\n\n\n\n\n\n\n==================");
                printf("Kubot.joy_cnt : %d",Kubot.joy_cnt);
                printf("Kubot.joy_FB_ref : %lf",Kubot.joy_FB_ref);
        }
    }


    }
    if(joy_msgs.buttons[1] == 1){ // 세모 : 14번
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
    if(joy_msgs.buttons[0] == 1){ // O : 2번
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");
    }
    if(joy_msgs.buttons[2] == 1){ // O : 일어나기(앞)
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_FRONT;
        printf("WALK READY START! \n");
    }
    if(joy_msgs.buttons[3] == 1){ // O : 일어나기(뒤)
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_BACK;
        printf("WALK READY START! \n");
    }
    if(joy_msgs.buttons[5] == 1){ // O : STOP
       if(Kubot.stop_msgs == 0){
       Kubot.stop_msgs = 1;
       printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");printf("Stop! \n");
       }
    }
    if(joy_msgs.buttons[6] == 1){ // O : convert_joy
        if(Kubot.joy_flag == true){
            Kubot.joy_flag = false;
        }
        else{
            Kubot.joy_flag = true;
        }
        printf("change_joy! now : %d\n", Kubot.joy_flag);
    }
    if(Kubot.Move_current == true){
    Kubot.FBSize_msgs = joy_msgs.axes[1]*Kubot.joy_FB_ref;
    Kubot.LRSize_msgs = joy_msgs.axes[0]*0.05;
    Kubot.TurnSize_msgs = joy_msgs.axes[3]*0.15;


    }
/////////////////////////////////////////////////////////    
    
}

void gazebo::kubot23_plugin::KubotMode(const std_msgs::Int32 &msg)
{
    // ROS_INFO("I heard: [%d]",msg.data);

    ControlMode_by_ROS = msg.data;
    
    switch (ControlMode_by_ROS) {

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
        Kubot.CommandFlag = SWAYMOTION_PATTERN;
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

    case 20:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_UPPER_TEST;
        Kubot.CommandFlag = ACT_UPPER_TEST;
        printf("UPPER BODY MOVE START! \n");
        

        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        break;

    case 21:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_FRONT;
        printf("STANDUP FRONT MOVE START! \n");
                
        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.STANDUP_READY = true;
        Kubot.STANDUP_ING = false;

        break;

    case 22:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_STANDUP_TEST;
        Kubot.CommandFlag = ACT_STANDUP_BACK;
        printf("STANDUP BACK MOVE START! \n");
                
        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.STANDUP_READY = true;
        Kubot.STANDUP_ING = false;

        break;

    case 31:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_SRCMOTION;
        Kubot.CommandFlag = ACT_HURDEL;
        printf("HURDEL MOTION START! \n");
                
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;

        break;

    case 32:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_SRCMOTION;
        Kubot.CommandFlag = ACT_BALLCATCH;
        printf("BALL CATCH MOTION START! \n");
                
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;

        break;

    case 33:
        Kubot.startTime = Kubot.realTime;
        Kubot.ControlMode = CTRLMODE_SRCMOTION;
        Kubot.CommandFlag = ACT_BALLSWING;
        printf("BALL SWING MOTION START! \n");
        
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        Kubot.MOTION_READY = true;
        Kubot.MOTION_ING = false;

        break;
        
    case 99://save file

        printf("save file START!\n");
        Kubot.save_file(50000);
        printf("save file DONE!\n");
        
        break;

    default:
         break;
    }

} 
void gazebo::kubot23_plugin::Kubot_control_callback(const kubot23w_pkgs::Kubot_control_msgs &msg){
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
    printf("steps : %d, FBSize : %lf, LRSize : %lf, TurnSize : %lf, footHeight : %lf, stop : %d" , Kubot.steps_msgs, Kubot.FBSize_msgs, Kubot.LRSize_msgs, Kubot.TurnSize_msgs, Kubot.footHeight_msgs,Kubot.stop_msgs);

}

void gazebo::kubot23_plugin::UpdateAlgorithm()
{

    //* UPDATE TIME : 1ms
    //printf("update_do_time\n");
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //        cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;
    Kubot.realTime =  Kubot.realTime + dt;
    //* setting for getting dt at next step
    last_update_time = current_time;
    
    getjointdata();
    getsensordata();

    static double steps    = 0;
    static double FBSize   = 0;
    static double LRSize   = 0;
    static double TurnSize = 0;

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

    VectorXd L_foot_FKcheck(6);
    VectorXd R_foot_FKcheck(6);


    //printf("update_do_getdata\n");
    // if(Kubot.Move_current == true){        
    //     VectorXd init_radian_po(12);
    //     // init_radian_po = Kubot.walkReadyAngle;
    //     for(int i=0;i<12;i++){

    //         joint[i].targetRadian = Kubot.walkReadyAngle[i];

    //     }

    // //     // VectorXd StandPose(12);
    // //     // StandPose=Kubot.IK_12(GB,LF,RF);
    // }
    //printf("update_do_init\n");


    static int con_count = 0;

    //* Real or simulated real-time thread time setting
    if (con_count % (int) (tasktime * 1000 + 0.001) == 0)
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

        case CTRLMODE_UPPER_TEST:
         
            break;

        case CTRLMODE_SRCMOTION:

            break;
        }


        //* CommandFlag
        switch (Kubot.CommandFlag) 
        {
        ///////////////////////////////////////////////////////////////////////////
        //////////////////////////////upper body FALG//////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        case ACT_UPPER_TEST:
            if (Kubot.Move_current == false) {
                Kubot.upperbodytest(5.0);
                for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====TEST COMPLETE !=====\n");
            }
            break;

        case ACT_STANDUP_FRONT: // up and down

            if (Kubot.STANDUP_ING == false) {
                Kubot.standupFront(1.0);

            }
            else {
                Kubot.STANDUP_ING = true;
                Kubot.CommandFlag = NONE_ACT;

                printf("=====STANDUP FRONT COMPLETE !=====\n");
            }
            break;    

        case ACT_STANDUP_BACK: // up and down

            if (Kubot.STANDUP_ING == false) {
                Kubot.standupBack(1.0);

            }
            else {
                Kubot.STANDUP_ING = true;
                Kubot.CommandFlag = NONE_ACT;
                printf("=====STANDUP BACK COMPLETE !=====\n");
            }
            break;  

        case ACT_HURDEL: // up and down

            if (Kubot.MOTION_ING == false) {
                Kubot.hurdlemotion(1.0);

            }
            else {
                Kubot.MOTION_ING = true;
                Kubot.ControlMode = CTRLMODE_WALKREADY;
                Kubot.CommandFlag = GOTO_WALK_READY_POS;
                printf("=====HURDEL MOTION COMPLETE !=====\n");
            }
            break;

        case ACT_BALLCATCH: // up and down

            if (Kubot.MOTION_ING == false) {
                Kubot.BallCatchMotion(2.5);

            }
            else {
                Kubot.MOTION_ING = true;
                Kubot.CommandFlag = NONE_ACT;
                printf("=====CATCH MOTION COMPLETE !=====\n");
            }
            break;   

        case ACT_BALLSWING: // up and down

            if (Kubot.MOTION_ING == false) {
                Kubot.BallSwingMotion(1.0);

            }
            else {
                Kubot.MOTION_ING = true;
                Kubot.CommandFlag = NONE_ACT;
                printf("=====SWING MOTION COMPLETE !=====\n");
            }
            break;  
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////    

        case GOTO_WALK_READY_POS: //down
            //WalkReady : Preparing the robot for walking
            if (Kubot.Move_current == false) {
                Kubot.walkingReady(5.0);

                for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("WALK_READY COMPLETE !\n");
            }
            break;

        case RETURN_HOMEPOSE: // up

            if (Kubot.Move_current == false) {

                Kubot.HomePose(5.0);
                 for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }
            }
            else {
                printf("RETURN_HOMEPOSE COMPLETE !\n");
                Kubot.ControlMode = CTRLMODE_HOMEPOSE;
                Kubot.CommandFlag = NONE_ACT;
            }
            break;

        case UPDOWN: // up and down

            if (Kubot.Move_current == true) {
                Kubot.ControlMode = CTRLMODE_UPDOWN;
                Kubot.CommandFlag = UPDOWN;
                printf("UPDOWN ING !\n");
            }
            
            break;       

        case SWAYMOTION_PATTERN: // sway motion pattern
            printf("pattern_start=====\n");
            if (Kubot.SWAY_ING == false) {
                Kubot.ControlMode = CTRLMODE_SWAY;
                printf("sway_start=====\n");

                printf("SWAY READY !\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("PATTERN READY COMPLETE !\n");
            }
            break;        

        case SWAYMOTION: // sway motion pattern

            if (Kubot.SWAY_READY = true) {
                Kubot.ControlMode = CTRLMODE_SWAY;


                // printf("SWAY ING !\n");
            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("SWAY COMPLETE !\n");
            }

            break;     

        case KICK_PATTERN: // kick motion pattern
            printf("=====pattern_start=====\n");
            if (Kubot.KICK_ING == false) {
                Kubot.ControlMode = CTRLMODE_KICKTEST;
                printf("=====KICK START=====\n");

                printf("=====KICK READY !=====\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break;        

        case KICKTEST: // kick motion pattern

            if (Kubot.KICK_READY = true) {
                Kubot.ControlMode = CTRLMODE_KICKTEST;


                // printf("KICK ING !\n");
            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====KICK COMPLETE !=====\n");
            }
            break;        

        case ACT_PATTERN_TEST: // walk motion pattern
            printf("=====WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false) {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====WALK READY !=====\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break;  

        case ACT_SIDE_PATTERN_TEST: // side motion pattern
            printf("=====SIDE WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false) {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====SIDE WALK READY !=====\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break; 

        case ACT_TURN_PATTERN_TEST: // turn motion pattern
            printf("=====TURN WALK pattern_start=====\n");
            if (Kubot.WALK_ING == false) {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

                printf("=====TURN WALK READY !=====\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====PATTERN READY COMPLETE !=====\n");
            }
            break; 

        case ACT_TEST_WALK: // walk motion pattern

            if (Kubot.WALK_READY = true) {
                Kubot.ControlMode = CTRLMODE_WALKING_TEST;

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("=====WALK COMPLETE !=====\n");
            }
            break;        


        case ACT_STOPPING_WALK:

            break;

        case ACT_STOP_WALK:

            break;

        case ACT_START_WALK:

            break;

        case ACT_INF_WALK:

            break;

        case NONE_ACT:
        FBSize = 0;
        LRSize = 0;
        TurnSize = 0;


        default:
        
            break;
        }
        // printf("==walking generate start =====\n");
        Kubot.walkingPatternGenerator(Kubot.ControlMode,Kubot.CommandFlag,Kubot.steps_msgs,Kubot.FBSize_msgs,Kubot.LRSize_msgs,Kubot.TurnSize_msgs,Kubot.footHeight_msgs,Kubot.stop_msgs);



        if (Kubot.ControlMode != CTRLMODE_WALKREADY) {

            VectorXd joint_IK(12);

            if (Kubot.Move_current == true && Kubot.CommandFlag == UPDOWN) {

                VectorXd GFL_Vector(6);
                GFL_Vector << 0, 0.05, 0, 0, 0, 0;
        
                VectorXd GFR_Vector(6);
                GFR_Vector << 0, -0.05, 0, 0, 0, 0;

                //수정
                // VectorXd FK_xyz = VectorXd::Zero(3);
                // FK_xyz = Kubot.jointToPosition(test_vector_1);
                
                // // std::cout << "\nFK_xyz:\n" << FK_xyz << std::endl;

                // double FK_z = -FK_xyz(2);
                // std::cout << "\nFK_z:\n" << FK_z << std::endl;

// double CKubot::cosWave(amp,  period,  time,  int_pos)
// {
//    return (amp / 2)*(1 - cos(PI / period * time)) + int_pos;
// } amp 목표값-현재값, 전체 시간, dt같은 느낌, 현재값

                // ref_G_CoM_z = 0.34 + (FK_z)/2 * (1 - cos(PI/T * dt));

                VectorXd GB_Vector(6);

                double ref_B_x = 0;
                double ref_B_y = 0;
                double ref_B_z; //Base 위치

                double z_max = 0.42;
                double z_min = 0.34;
                double T = 1000;

                ref_B_z = Kubot.cosWave(z_max-z_min, T, Kubot.updown_cnt, z_min);
                Kubot.updown_cnt = Kubot.updown_cnt + 1;

                std::cout << "\nref_B_z:\n" << ref_B_z << std::endl;

                GB_Vector << ref_B_x, ref_B_y, ref_B_z, 0, 0, 0;
                

                
                joint_IK << Kubot.Geometric_IK_L(GB_Vector, GFL_Vector), Kubot.Geometric_IK_R(GB_Vector, GFR_Vector);
            }                   
            

                   // sway 실행 
            else if(Kubot.CommandFlag == SWAYMOTION && Kubot.SWAY_READY == true) {

                    VectorXd GBS_Vector(6);

                    VectorXd GFL_Vector(6);
                    GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                    VectorXd GFR_Vector(6);
                    GFR_Vector << 0, -0.05, 0, 0, 0, 0;

                    GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.34, 0, 0, 0;
                    
                    Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;



                    joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
                        // m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
                        // P_preview_COM_y.publish(m_preview_COM_y);

                        //FK 계산출력 식//
                        // VectorXd zmpFKcheck(3);


                        // zmpFKcheck << Kubot.ZMPFK(test_vector_1);
                        // std::cout << "\nzmpFK:\n" << zmpFKcheck << std::endl;
                        // zmpFK_Y = zmpFKcheck(1);

            }

                // 두 발자국
            else if(Kubot.CommandFlag == KICKTEST && Kubot.KICK_READY == true) {


                    VectorXd GBS_Vector(6);

                    VectorXd GFL_Vector(6);

                    VectorXd GFR_Vector(6);
                    double ref_RF_x;
                    double ref_RF_z;

                    double RFx1_max = 0.05;
                    double RFx1_min = 0.0;

                    double RFx2_max = 0.1;
                    double RFx2_min = 0.05;

                    double RFz_max = 0.06;
                    double RFz_min = 0.0;

                    double T = 2000;

                    GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.38, 0, 0, 0;

                   


                    if (Kubot.zmp.previewControl.count < 3000)
                    {
                        GFR_Vector << 0, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0, 0.05, 0, 0, 0, 0;
                    }

                    else if (Kubot.zmp.previewControl.count >= 3000 && Kubot.zmp.previewControl.count < 5000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-3000, RFz_min);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0, -0.05, ref_RF_z, 0, 0, 0;
                        GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                    }


                    else if (Kubot.zmp.previewControl.count >= 5000 && Kubot.zmp.previewControl.count < 7000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_x = Kubot.cosWave(RFx1_max-RFx1_min, T, Kubot.zmp.previewControl.count-5000, RFx1_min);


                        std::cout << "\nref_RB_x:\n" << ref_RF_x << std::endl;
                        GFR_Vector << ref_RF_x, -0.05, RFz_max, 0, 0, 0;
                        GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                    }

                    else if (Kubot.zmp.previewControl.count >= 7000 && Kubot.zmp.previewControl.count < 9000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-7000, RFz_max);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << RFx1_max, -0.05, ref_RF_z, 0, 0, 0;
                        GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if(Kubot.zmp.previewControl.count >= 9000 && Kubot.zmp.previewControl.count < 10000)
                    {
                        GFR_Vector << 0.05, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0, 0.05, 0, 0, 0, 0;

                    }

                    else if(Kubot.zmp.previewControl.count >= 10000 && Kubot.zmp.previewControl.count < 12000)
                    {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-10000, RFz_min);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0.05, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0, 0.05, ref_RF_z, 0, 0, 0;
                    }


                    else if (Kubot.zmp.previewControl.count >= 12000 && Kubot.zmp.previewControl.count < 14000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_x = Kubot.cosWave(RFx1_max-RFx1_min, T, Kubot.zmp.previewControl.count-12000, RFx1_min);

                        std::cout << "\nref_RB_x:\n" << ref_RF_x << std::endl;
                        GFR_Vector << 0.05, -0.05, 0, 0, 0, 0;
                        GFL_Vector << ref_RF_x, 0.05, RFz_max, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if (Kubot.zmp.previewControl.count >= 14000 && Kubot.zmp.previewControl.count < 16000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-14000, RFz_max);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0.05, -0.05, 0, 0, 0, 0;
                        GFL_Vector << RFx1_max, 0.05, ref_RF_z, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if (Kubot.zmp.previewControl.count >= 16000 && Kubot.zmp.previewControl.count < 17000)
                    {
                        GFR_Vector << 0.05, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;

                    }

                    //////////////////////////////////////////////////////////////////////////////
                    //////////////////////////////////////////////////////////////////////////////
                    //////////////////////////////////////////////////////////////////////////////

                    else if (Kubot.zmp.previewControl.count >= 17000 && Kubot.zmp.previewControl.count < 19000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-17000, RFz_min);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0.05, -0.05, ref_RF_z, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }



                    else if (Kubot.zmp.previewControl.count >= 19000 && Kubot.zmp.previewControl.count < 21000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_x = Kubot.cosWave(RFx2_max-RFx2_min, T, Kubot.zmp.previewControl.count-19000, RFx2_min);

                        std::cout << "\nref_RB_x:\n" << ref_RF_x << std::endl;
                        GFR_Vector << ref_RF_x, -0.05, RFz_max, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if (Kubot.zmp.previewControl.count >= 21000 && Kubot.zmp.previewControl.count < 23000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-21000, RFz_max);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << RFx2_max, -0.05, ref_RF_z, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if(Kubot.zmp.previewControl.count >= 23000 && Kubot.zmp.previewControl.count < 24000)
                    {
                        GFR_Vector << 0.1, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, 0, 0, 0, 0;

                    }

                    else if(Kubot.zmp.previewControl.count >= 24000 && Kubot.zmp.previewControl.count < 26000)
                    {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_max-RFz_min, T, Kubot.zmp.previewControl.count-24000, RFz_min);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0.1, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0.05, 0.05, ref_RF_z, 0, 0, 0;
                    }


                    else if (Kubot.zmp.previewControl.count >= 26000 && Kubot.zmp.previewControl.count < 28000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_x = Kubot.cosWave(RFx2_max-RFx2_min, T, Kubot.zmp.previewControl.count-26000, RFx2_min);

                        std::cout << "\nref_RB_x:\n" << ref_RF_x << std::endl;
                        GFR_Vector << 0.1, -0.05, 0, 0, 0, 0;
                        GFL_Vector << ref_RF_x, 0.05, RFz_max, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if (Kubot.zmp.previewControl.count >= 28000 && Kubot.zmp.previewControl.count < 30000) {
                        printf("==============@@@@@@@@@@@@@@@@@@@@@@@@=============");

                        ref_RF_z = Kubot.cosWave(RFz_min-RFz_max, T, Kubot.zmp.previewControl.count-28000, RFz_max);

                        std::cout << "\nref_RB_z:\n" << ref_RF_z << std::endl;
                        GFR_Vector << 0.1, -0.05, 0, 0, 0, 0;
                        GFL_Vector << RFx2_max, 0.05, ref_RF_z, 0, 0, 0;

                    // zmp.previewControl.count = zmp.previewControl.count + 1;
                    }

                    else if(Kubot.zmp.previewControl.count >= 30000)
                    {
                        GFR_Vector << 0.1, -0.05, 0, 0, 0, 0;
                        GFL_Vector << 0.1, 0.05, 0, 0, 0, 0;

                    }


                    L_foot_ref_x = GFL_Vector(0);
                    L_foot_ref_z = GFL_Vector(2);
                    R_foot_ref_x = GFR_Vector(0);
                    R_foot_ref_z = GFR_Vector(2);


                    joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
                    Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;
                        // m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
                        // P_preview_COM_y.publish(m_preview_COM_y);

                        //FK 계산출력 식//
                        // VectorXd zmpFKcheck(3);



                        L_foot_FKcheck << Kubot.jointToPosition(test_vector, GBS_Vector(0),GBS_Vector(1) ,GBS_Vector(2) );

                        L_foot_FK_x = L_foot_FKcheck(0);
                        L_foot_FK_z = L_foot_FKcheck(2);

                        // R_foot_FKcheck << Kubot.jointToPosition_R(test_vector);

                        R_foot_FK_x = L_foot_FKcheck(3);
                        R_foot_FK_z = L_foot_FKcheck(5);

                        // FK 계산출력 식//
                        VectorXd zmpFKcheck(3);


                        zmpFKcheck << Kubot.ZMPFK(test_vector, L_foot_FKcheck);
                        std::cout << "\nzmpFK:\n" << zmpFKcheck << std::endl;
                        zmpFK_X = zmpFKcheck(0);
                        zmpFK_Y = zmpFKcheck(1);


            }

            // foot traj generate and walking
            else if(Kubot.CommandFlag == ACT_TEST_WALK || Kubot.CommandFlag == ACT_INF_WALK || Kubot.CommandFlag == ACT_STOP_WALK && Kubot.WALK_READY == true) {

                    VectorXd GBS_Vector(6);

                    VectorXd GFL_Vector(6);
                    VectorXd GFR_Vector(6);


                    GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.38, 0, 0, Kubot.Base.refpos(Kubot.Yaw);
                    // std::cout << "\nbase Yaw\n" << GBS_Vector(5) << std::endl;


                    GFL_Vector << Kubot.LFoot.ref_G_pattern_pos(0), Kubot.LFoot.ref_G_pattern_pos(1), Kubot.LFoot.ref_G_pattern_pos(2), 0, 0, Kubot.LFoot.refpos(5);
 
                    GFR_Vector << Kubot.RFoot.ref_G_pattern_pos(0), Kubot.RFoot.ref_G_pattern_pos(1), Kubot.RFoot.ref_G_pattern_pos(2), 0, 0, Kubot.RFoot.refpos(5);

                    // std::cout << "\nKubot.zmp.previewControl.count:\n" << Kubot.zmp.previewControl.count << std::endl;
                    // std::cout << "\nGFL_Vector:\n" << GFL_Vector(5) << std::endl;
                    // std::cout << "\nGFR_Vector:\n" << GFR_Vector(5) << std::endl;

                    L_foot_ref_x = GFL_Vector(0);
                    L_foot_ref_y = GFL_Vector(1);
                    L_foot_ref_z = GFL_Vector(2);
                    
                    R_foot_ref_x = GFR_Vector(0);
                    R_foot_ref_y = GFR_Vector(1);
                    R_foot_ref_z = GFR_Vector(2);


                    joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
                    // std::cout << "\njoint_IK\n" << joint_IK << std::endl;


                    L_foot_FKcheck << Kubot.jointToPosition(test_vector, GBS_Vector(0),GBS_Vector(1) ,GBS_Vector(2) );

                        L_foot_FK_x = L_foot_FKcheck(0);
                        L_foot_FK_y = L_foot_FKcheck(1);
                        L_foot_FK_z = L_foot_FKcheck(2);

                        R_foot_FK_x = L_foot_FKcheck(3);
                        R_foot_FK_y = L_foot_FKcheck(4);
                        R_foot_FK_z = L_foot_FKcheck(5);

                        // FK 계산출력 식//
                        VectorXd zmpFKcheck(3);

                        zmpFKcheck << Kubot.ZMPFK(test_vector, L_foot_FKcheck);
                        // std::cout << "\nzmpFK:\n" << zmpFKcheck << std::endl;
                        zmpFK_X = zmpFKcheck(0);
                        zmpFK_Y = zmpFKcheck(1);
                        zmpFK_Z = zmpFKcheck(2);


            }
            

            if (Kubot.ControlMode == CTRLMODE_HOMEPOSE || Kubot.ControlMode == CTRLMODE_STANDUP_TEST || Kubot.ControlMode == CTRLMODE_SRCMOTION)
            {
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

                joint[LSP].targetRadian = Kubot.refAngle[12];     // L_shoulder_pitch_joint 플러스가 어깨 뒤로
                joint[LER].targetRadian = Kubot.refAngle[13];     // L_elbow_roll_joint 플러스가 좌우로 나란히
                joint[LHA].targetRadian = Kubot.refAngle[14];     // L_hand_pitch_joint  플러스가 안으로 굽
                joint[RSP].targetRadian = Kubot.refAngle[15];     // R_shoulder_pitch_joint
                joint[RER].targetRadian = Kubot.refAngle[16];     // R_elbow_roll_joint
                joint[RHA].targetRadian = Kubot.refAngle[17];     // R_hand_pitch_joint
                joint[NYA].targetRadian = Kubot.refAngle[18];     // Neck_yaw_joint
                joint[HEP].targetRadian = Kubot.refAngle[19];     // head_pitch_joint
                    // printf("==== ControlMode check ====\n");
            }
            else if(Kubot.CommandFlag == ACT_INF_WALK || Kubot.CommandFlag == ACT_STOP_WALK)
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
                joint[LSP].targetRadian = Kubot.refAngle[12];     // L_shoulder_pitch_joint 플러스가 어깨 뒤로
                joint[LER].targetRadian = Kubot.refAngle[13];     // L_elbow_roll_joint 플러스가 좌우로 나란히
                joint[LHA].targetRadian = Kubot.refAngle[14];     // L_hand_pitch_joint  플러스가 안으로 굽
                joint[RSP].targetRadian = Kubot.refAngle[15];     // R_shoulder_pitch_joint
                joint[RER].targetRadian = Kubot.refAngle[16];     // R_elbow_roll_joint
                joint[RHA].targetRadian = Kubot.refAngle[17];     // R_hand_pitch_joint
                joint[NYA].targetRadian = Kubot.refAngle[18];     // Neck_yaw_joint
                joint[HEP].targetRadian = Kubot.refAngle[19];     // head_pitch_joint
                // printf("LLL = %lf\n", joint[LHY].targetRadian);
                // printf("RRR = %lf\n", joint[RHY].targetRadian);

            }

        }

}




    // Kubot.abc(); //Kubot class안에 만든 abc 실행
    // Kubot.IK();

    jointcontroller();

    if(Kubot.Move_current == true){
        int timeCnt;
        timeCnt = (int)((Kubot.realTime - Kubot.startTime)*1000 + 0.001); //[msec]
            
            Kubot.save_jointData[0][timeCnt] = timeCnt;
            for(int i = 2;i<=13;i++){
                Kubot.save_jointData[i-1][timeCnt] = joint[i-2].targetRadian;
            }
            for(int i = 14;i<=25;i++){
                Kubot.save_jointData[i-1][timeCnt] = joint[i-14].actualRadian;
            }
        
        
        Kubot.save_previewData[0][timeCnt] = timeCnt;
        Kubot.save_previewData[1][timeCnt] = Kubot.zmp.previewControl.X.m_ref;// ZMP_ref_X 
        Kubot.save_previewData[2][timeCnt] = Kubot.zmp.previewControl.Y.m_ref;// ZMP_ref_Y 
        Kubot.save_previewData[3][timeCnt] = Kubot.zmp.previewControl.X.old_zmp;// ZMP_calculate_X 
        Kubot.save_previewData[4][timeCnt] = Kubot.zmp.previewControl.Y.old_zmp;// ZMP_calculate_Y
        Kubot.save_previewData[5][timeCnt] = Kubot.zmp.previewControl.X.CoM;// Com_X
        Kubot.save_previewData[6][timeCnt] = Kubot.zmp.previewControl.Y.CoM;// Com_Y
        Kubot.save_previewData[7][timeCnt] = 0.34;// Com_Z
        Kubot.save_previewData[8][timeCnt] = zmpFK_X;// FK_X
        Kubot.save_previewData[9][timeCnt] = zmpFK_Y;// FK_Y
        Kubot.save_previewData[10][timeCnt] = zmpFK_Z;// FK_Z
      
        Kubot.save_footData[0][timeCnt] = timeCnt;
        Kubot.save_footData[1][timeCnt] = L_foot_ref_x; //L_ref_X
        Kubot.save_footData[2][timeCnt] = L_foot_ref_y; //L_ref_Y
        Kubot.save_footData[3][timeCnt] = L_foot_ref_z; //L_ref_Z
        Kubot.save_footData[4][timeCnt] = R_foot_ref_x; //R_ref_X
        Kubot.save_footData[5][timeCnt] = R_foot_ref_y; //R_ref_Y
        Kubot.save_footData[6][timeCnt] = R_foot_ref_z; //R_ref_Z
        Kubot.save_footData[7][timeCnt] = L_foot_FK_x; //L_FK_X
        Kubot.save_footData[8][timeCnt] = L_foot_FK_y; //L_FK_Y
        Kubot.save_footData[9][timeCnt] = L_foot_FK_z; //L_FK_Z
        Kubot.save_footData[10][timeCnt] = R_foot_FK_x; //R_FK_X
        Kubot.save_footData[11][timeCnt] = R_foot_FK_y; //R_FK_Y
        Kubot.save_footData[12][timeCnt] = R_foot_FK_z; //R_FK_Z

		// printf("data save // time : %d \n ",timeCnt);


    //////////////base//////////////

    m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
    P_preview_COM_y.publish(m_preview_COM_y);

    m_preview_ref_ZMP_y.data = Kubot.zmp.previewControl.Y.m_ref;
    P_preview_ref_ZMP_y.publish(m_preview_ref_ZMP_y);

    m_preview_COM_x.data = Kubot.zmp.previewControl.X.CoM;
    P_preview_COM_x.publish(m_preview_COM_x);

    m_preview_ref_ZMP_x.data = Kubot.zmp.previewControl.X.m_ref;
    P_preview_ref_ZMP_x.publish(m_preview_ref_ZMP_x);

    m_ZMP_x.data = Kubot.zmp.previewControl.X.old_zmp;
    P_ZMP_x.publish(m_ZMP_x);

    m_ZMP_y.data = Kubot.zmp.previewControl.Y.old_zmp;
    P_ZMP_y.publish(m_ZMP_y);

    m_preview_FK_x.data = zmpFK_X;
    P_preview_FK_x.publish(m_preview_FK_x);
    m_preview_FK_y.data = zmpFK_Y;
    P_preview_FK_y.publish(m_preview_FK_y);
    
    //////////////L_foot//////////////
    m_L_foot_ref_x.data = L_foot_ref_x;
    P_L_foot_ref_x.publish(m_L_foot_ref_x);

    m_L_foot_ref_z.data = L_foot_ref_z;
    P_L_foot_ref_z.publish(m_L_foot_ref_z);

    m_L_foot_FK_x.data = L_foot_FK_x;
    P_L_foot_FK_x.publish(m_L_foot_FK_x);

    m_L_foot_FK_z.data = L_foot_FK_z;
    P_L_foot_FK_z.publish(m_L_foot_FK_z);

    //////////////R_foot//////////////
    m_R_foot_ref_x.data = R_foot_ref_x;
    P_R_foot_ref_x.publish(m_R_foot_ref_x);

    m_R_foot_ref_z.data = R_foot_ref_z;
    P_R_foot_ref_z.publish(m_R_foot_ref_z);

    m_R_foot_FK_x.data = R_foot_FK_x;
    P_R_foot_FK_x.publish(m_R_foot_FK_x);

    m_R_foot_FK_z.data = R_foot_FK_z;
    P_R_foot_FK_z.publish(m_R_foot_FK_z);



    KubotModesp_pub.publish(KubotModesp_msg); 
  
    }
}
 



void gazebo::kubot23_plugin::setjoints() //plugin에다가 joints name 설정 .sdf파일에서 설정한 이름이랑 확인하기
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");

    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");

    L_shoulder_pitch_joint = this->model->GetJoint("L_shoulder_pitch_joint");
    L_elbow_roll_joint = this->model->GetJoint("L_elbow_roll_joint");
    L_hand_pitch_joint = this->model->GetJoint("L_hand_pitch_joint");

    R_shoulder_pitch_joint = this->model->GetJoint("R_shoulder_pitch_joint");
    R_elbow_roll_joint = this->model->GetJoint("R_elbow_roll_joint");
    R_hand_pitch_joint = this->model->GetJoint("R_hand_pitch_joint");

    Neck_yaw_joint = this->model->GetJoint("Neck_yaw_joint");
    head_pitch_joint = this->model->GetJoint("head_pitch_joint");

    //L_Foot_joint = this->model->GetJoint("L_Foot_joint");
    //R_Foot_joint = this->model->GetJoint("R_Foot_joint");
}

void gazebo::kubot23_plugin::getjointdata()
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
    //printf("before for \n");
    joint[LSP].actualRadian = L_shoulder_pitch_joint->Position(0);
    joint[LER].actualRadian = L_elbow_roll_joint->Position(0);
    joint[LHA].actualRadian = L_hand_pitch_joint->Position(0);

    joint[RSP].actualRadian = R_shoulder_pitch_joint->Position(0);
    joint[RER].actualRadian = R_elbow_roll_joint->Position(0);
    joint[RHA].actualRadian = R_hand_pitch_joint->Position(0);

    joint[NYA].actualRadian = Neck_yaw_joint->Position(0);
    joint[HEP].actualRadian = head_pitch_joint->Position(0);
    

    for (int j = 0; j < 20; j++) { //ndof -> 12
        joint[j].actualDegree = joint[j].actualRadian*R2D;
        
        test_vector(j) = joint[j].actualRadian;
        //printf("update_encorder\n");
        //Kubot.actual_radian_vector(j) = joint[j].actualRadian;
        //printf("vector_data : %lf \n", Kubot.actual_radian_vector(j));
        // printf(C_BLUE "joint[%d].actualRadian = %f\n" C_RESET, j, joint[j].actualRadian);
    }



    //printf("complete for\n");
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

    joint[LSP].actualVelocity = L_shoulder_pitch_joint->GetVelocity(0);
    joint[LER].actualVelocity = L_elbow_roll_joint->GetVelocity(0);
    joint[LHA].actualVelocity = L_hand_pitch_joint->GetVelocity(0);

    joint[RSP].actualVelocity = R_shoulder_pitch_joint->GetVelocity(0);
    joint[RER].actualVelocity = R_elbow_roll_joint->GetVelocity(0);
    joint[RHA].actualVelocity = R_hand_pitch_joint->GetVelocity(0);

    joint[NYA].actualVelocity = Neck_yaw_joint->GetVelocity(0);
    joint[HEP].actualVelocity = head_pitch_joint->GetVelocity(0);

    for (int j = 0; j < nDoF; j++) {

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

    joint[LSP].actualTorque = L_shoulder_pitch_joint->GetForce(0);
    joint[LER].actualTorque = L_elbow_roll_joint->GetForce(0);
    joint[LHA].actualTorque = L_hand_pitch_joint->GetForce(0);

    joint[RSP].actualTorque = R_shoulder_pitch_joint->GetForce(0);
    joint[RER].actualTorque = R_elbow_roll_joint->GetForce(0);
    joint[RHA].actualTorque = R_hand_pitch_joint->GetForce(0);

    joint[NYA].actualTorque = Neck_yaw_joint->GetForce(0);
    joint[HEP].actualTorque = head_pitch_joint->GetForce(0);

    for (int j = 0; j < nDoF; j++) {

        joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    }  

}


void gazebo::kubot23_plugin::setsensor() //sdf파일에 있는 sensor를 설정하는 함수
{
        Sensor = sensors::get_sensor("IMU");
        IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);   
}

void gazebo::kubot23_plugin::getsensordata()
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



void gazebo::kubot23_plugin::jointcontroller()
{
    // double amp = 1;
    // double period = 3.0; // 이동하는 데 걸리는 시간 (주기)
    // double int_pos = 0.0;

    // double target_angle = cosWave(amp, period, time, int_pos);

    
    //     for (int j = 0; j < nDoF; j++) {

    //     joint[j].targetRadian = target_angle*joint[j].init_targetradian;
    //     joint[j].targetDegree = joint[j].targetRadian * R2D;
    //     joint[j].targetVelocity = (joint[j].targetRadian - pre_rad[j]) / dt;
    //     pre_rad[j] = joint[j].targetRadian;

    //     joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) \
    //                           + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));
    //     }
    

    
        for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) \
                              + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));
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

        L_shoulder_pitch_joint->SetForce(0, joint[LSP].targetTorque);        
        L_elbow_roll_joint->SetForce(0, joint[LER].targetTorque);         
        L_hand_pitch_joint->SetForce(0, joint[LHA].targetTorque);

        R_shoulder_pitch_joint->SetForce(0, joint[RSP].targetTorque);
        R_elbow_roll_joint->SetForce(0, joint[RER].targetTorque);
        R_hand_pitch_joint->SetForce(0, joint[RHA].targetTorque); 

        Neck_yaw_joint->SetForce(0, joint[NYA].targetTorque);        
        head_pitch_joint->SetForce(0, joint[HEP].targetTorque); 

   
        //함수만 선언해놓고 함수를 실행안해서 아직은 실행불가
        //여기서 안돌아가는데 sdf파일 lower / upper 값 수정 필요 주석처리로 제한해제
        //괄호안을 0->2로 수정 x,y,z 순서라 그런가...



}




void gazebo::kubot23_plugin::initializejoint()
{
    /*
     * Initialize joint variables for joint control
     */


        joint[0].init_targetradian  = (0  *D2R);     // L_Hip_yaw_joint
        joint[1].init_targetradian  = (0  *D2R);     // L_Hip_roll_joint
        joint[2].init_targetradian  = (-45*D2R);     // L_Hip_pitch_joint
        joint[3].init_targetradian  = (90 *D2R);     // L_Knee_joint
        joint[4].init_targetradian  = (-45*D2R);     // L_Ankle_pitch_joint
        joint[5].init_targetradian  = (0  *D2R);     // L_Ankle_roll_joint
        joint[6].init_targetradian  = (0  *D2R);     // R_Hip_yaw_joint
        joint[7].init_targetradian  = (0  *D2R);     // R_Hip_roll_joint
        joint[8].init_targetradian  = (-45*D2R);     // R_Hip_pitch_joint
        joint[9].init_targetradian  = (90 *D2R);     // R_Knee_joint
        joint[10].init_targetradian = (-45*D2R);     // R_Ankle_pitch_joint
        joint[11].init_targetradian = (0  *D2R);     // R_Ankle_roll_joint

        joint[12].init_targetradian = (0  *D2R);     // L_shoulder_pitch_joint
        joint[13].init_targetradian = (-15*D2R);     // L_elbow_roll_joint
        joint[14].init_targetradian = (90 *D2R);     // L_hand_pitch_joint
        joint[15].init_targetradian = (0  *D2R);     // R_shoulder_pitch_joint
        joint[16].init_targetradian = (-15*D2R);     // R_elbow_roll_joint
        joint[17].init_targetradian = (90 *D2R);     // R_hand_pitch_joint
        joint[18].init_targetradian = (-0 *D2R);     // Neck_yaw_joint
        joint[19].init_targetradian = (0  *D2R);     // head_pitch_joint
    // for (int j = 0; j < nDoF; j++) {
    //     joint[j].targetDegree = 0;
    //     joint[j].targetRadian = 0;
    //     joint[j].targetVelocity = 0;
    //     joint[j].targetTorque = 0;
        
    //     joint[j].actualDegree = 0;
    //     joint[j].actualRadian = 0;
    //     joint[j].actualVelocity = 0;
    //     joint[j].actualRPM = 0;
    //     joint[j].actualTorque = 0;
    // }
}


void gazebo::kubot23_plugin::setjointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
        joint[LHY].Kp = 130;
        joint[LHR].Kp = 140;
        joint[LHP].Kp = 160;
        joint[LKN].Kp = 230;
        joint[LAP].Kp = 130;
        joint[LAR].Kp = 130;

        joint[RHY].Kp = joint[LHY].Kp;
        joint[RHR].Kp = joint[LHR].Kp;
        joint[RHP].Kp = joint[LHP].Kp;
        joint[RKN].Kp = joint[LKN].Kp;
        joint[RAP].Kp = joint[LAP].Kp;
        joint[RAR].Kp = joint[LAR].Kp;

        joint[LHY].Kd = 0.06;
        joint[LHR].Kd = 0.08;
        joint[LHP].Kd = 0.11;
        joint[LKN].Kd = 0.10;
        joint[LAP].Kd = 0.06;
        joint[LAR].Kd = 0.06;

        joint[RHY].Kd = joint[LHY].Kd;
        joint[RHR].Kd = joint[LHR].Kd;
        joint[RHP].Kd = joint[LHP].Kd;
        joint[RKN].Kd = joint[LKN].Kd;
        joint[RAP].Kd = joint[LAP].Kd;
        joint[RAR].Kd = joint[LAR].Kd;

        joint[LSP].Kp = 10;
        joint[LER].Kp = 10;
        joint[LHA].Kp = 10;
        joint[RSP].Kp = 10;
        joint[RER].Kp = 10;
        joint[RHA].Kp = 10;
        joint[NYA].Kp = 10;
        joint[HEP].Kp = 10;

        joint[LSP].Kd = 0.01;
        joint[LER].Kd = 0.01;
        joint[LHA].Kd = 0.01;
        joint[RSP].Kd = 0.01;
        joint[RER].Kd = 0.01;
        joint[RHA].Kd = 0.01;
        joint[NYA].Kd = 0.01;
        joint[HEP].Kd = 0.01;

}

