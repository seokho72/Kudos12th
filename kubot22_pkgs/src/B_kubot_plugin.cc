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

    class B_kubot_plugin : public ModelPlugin
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

        enum
        { //wst는 torso joint 같음
            LHY = 0, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
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
        sensors::ImuSensorPtr IMU; //imu 센서 추가 (Z-Y-X 순서=gazebo와 같음)

    public :
            //*** Functions for Kubot Simulation in Gazebo ***//
          void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
          void UpdateAlgorithm(); // Algorithm update while simulation
          
          void getJoints(); // Get each joint data from [physics::ModelPtr _model]    
          void getjointData(); // Get encoder data of each joint
          
		  void getsensor(); // Get each sensor data from [sensors::SensorPtr Sensor]
          void getsensorData(); // Gen encoder data of imu sensor
  
          void jointcontroller(); // Joint Controller for each joint


        //   void initializejoint(); // 추가한
        //   void setjointPIDgain(); // 함수
    };
    GZ_REGISTER_MODEL_PLUGIN(B_kubot_plugin);
}

void gazebo::B_kubot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    getJoints();
    //sdf변환하기 전에 urdf파일에서 조인트가 몇개인지 인식을 함.
    //인식이 끝나면 joint 갯수에 맞게 구조체를 여러개 생성
    //Robotjoint 구조체를 joint 갯수만큼 복제되는 거임
    // getsensor(); //sdf파일에 있는 imu센서 부르기


    nDoF = 12; // Get degrees of freedom, except position and orientation of the robot
    //우리가 만든 joint는 12개로 지정
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct    

    // initializejoint();
    // setjointPIDgain();
    //* setting for getting dt
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&B_kubot_plugin::UpdateAlgorithm, this));
    
}

void gazebo::B_kubot_plugin::UpdateAlgorithm()
{
    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //        cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;
    
    getjointData();
    // printf(C_BLUE "time = %f\n" C_RESET, time);
    // getsensorData();

    jointcontroller();        
}

void gazebo::B_kubot_plugin::getJoints() //plugin에다가 joints name 설정 .sdf파일에서 설정한 이름이랑 확인하기
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

}

void gazebo::B_kubot_plugin::getjointData()
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

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
        printf(C_BLUE "joint[%d].actualDegree = %f\n" C_RESET, j, joint[j].actualDegree);
    }

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

    for (int j = 0; j < nDoF; j++) {

        joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    }

}

void gazebo::B_kubot_plugin::jointcontroller() //joint에 토크를 가하는 함수
{
        double kp, kd;

        double target_radian[12];

        target_radian[0] = 0*D2R;
        target_radian[1] = 0*D2R;
        target_radian[2] = -45*D2R;
        target_radian[3] = 90*D2R;
        target_radian[4] = -45*D2R;
        target_radian[5] = 0*D2R;
        target_radian[6] = 0*D2R;
        target_radian[7] = 0*D2R;
        target_radian[8] = -45*D2R;
        target_radian[9] = 90*D2R;
        target_radian[10] = -45*D2R;
        target_radian[11] = 0*D2R;

        kp = 10;
        kd = 0.1;

        for (int j = 0; j < nDoF; j++){

            joint[j].targetTorque = kp*(target_radian[j]-joint[j].actualRadian) \
                                  + kd*(-joint[j].actualVelocity);

        }               

        //* Update target torque in gazebo simulation
        L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
        L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
        L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
        L_Knee_joint->SetForce(0, joint[LKN].targetTorque); //LHY에 10N 저장해놓고 joint에 그 힘을 준다.
        L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);        
        L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);         

        R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
        R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
        R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
        R_Knee_joint->SetForce(0, joint[RKN].targetTorque); //LHY에 10N 저장해놓고 joint에 그 힘을 준다.
        R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);        
        R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque); 

   
        //함수만 선언해놓고 함수를 실행안해서 아직은 실행불가
        //여기서 안돌아가는데 sdf파일 lower / upper 값 수정 필요 주석처리로 제한해제
        //괄호안을 0->2로 수정 x,y,z 순서라 그런가...
        
}

/*
void gazebo::B_kubot_plugin::getsensor() //sdf파일에 있는 sensor를 설정하는 함수
{
        Sensor = sensors::get_sensor("IMU");
        IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);   
}

void gazebo::B_kubot_plugin::getsensorData()
{
         // IMU sensor data
        double IMUdata[3];

        IMUdata[0] = IMU->Orientation().Euler()[0];
        IMUdata[1] = IMU->Orientation().Euler()[1];
        IMUdata[2] = IMU->Orientation().Euler()[2];

        printf(C_BLUE "IMU roll = %f\n" C_RESET, IMUdata[0]*R2D );
        printf(C_RED "IMU pitch = %f\n" C_RESET, IMUdata[1]*R2D );
        printf(C_YELLOW "IMU yaw = %f\n" C_RESET, IMUdata[2]*R2D );
        

        // IMU_dtheta[Roll] = IMU->AngularVelocity(false)[Roll];
        // IMU_dtheta[Pitch] = IMU->AngularVelocity(false)[Pitch];
        // IMU_dtheta[Yaw] = IMU->AngularVelocity(false)[Yaw];

}
*/

// void gazebo::B_kubot_plugin::initializejoint()
// {
//     /*
//      * Initialize joint variables for joint control
//      */


//         joint[0].init_targetradian  = (0  *D2R);     // L_Hip_yaw_joint
//         joint[1].init_targetradian  = (0  *D2R);     // L_Hip_roll_joint
//         joint[2].init_targetradian  = (-45*D2R);     // L_Hip_pitch_joint
//         joint[3].init_targetradian  = (90 *D2R);     // L_Knee_joint
//         joint[4].init_targetradian  = (-45*D2R);     // L_Ankle_pitch_joint
//         joint[5].init_targetradian  = (0  *D2R);     // L_Ankle_roll_joint
//         joint[6].init_targetradian  = (0  *D2R);     // R_Hip_yaw_joint
//         joint[7].init_targetradian  = (0  *D2R);     // R_Hip_roll_joint
//         joint[8].init_targetradian  = (-45*D2R);     // R_Hip_pitch_joint
//         joint[9].init_targetradian  = (90 *D2R);     // R_Knee_joint
//         joint[10].init_targetradian = (-45*D2R);     // R_Ankle_pitch_joint
//         joint[11].init_targetradian = (0  *D2R);     // R_Ankle_roll_joint
//     // for (int j = 0; j < nDoF; j++) {
//     //     joint[j].targetDegree = 0;
//     //     joint[j].targetRadian = 0;
//     //     joint[j].targetVelocity = 0;
//     //     joint[j].targetTorque = 0;
        
//     //     joint[j].actualDegree = 0;
//     //     joint[j].actualRadian = 0;
//     //     joint[j].actualVelocity = 0;
//     //     joint[j].actualRPM = 0;
//     //     joint[j].actualTorque = 0;
//     // }
// }


// void gazebo::B_kubot_plugin::setjointPIDgain()
// {
//     /*
//      * Set each joint PID gain for joint control
//      */
//         joint[LHY].Kp = 100;
//         joint[LHR].Kp = 110;
//         joint[LHP].Kp = 130;
//         joint[LKN].Kp = 200;
//         joint[LAP].Kp = 100;
//         joint[LAR].Kp = 100;

//         joint[RHY].Kp = joint[LHY].Kp;
//         joint[RHR].Kp = joint[LHR].Kp;
//         joint[RHP].Kp = joint[LHP].Kp;
//         joint[RKN].Kp = joint[LKN].Kp;
//         joint[RAP].Kp = joint[LAP].Kp;
//         joint[RAR].Kp = joint[LAR].Kp;

//         joint[LHY].Kd = 0.03;
//         joint[LHR].Kd = 0.05;
//         joint[LHP].Kd = 0.08;
//         joint[LKN].Kd = 0.07;
//         joint[LAP].Kd = 0.03;
//         joint[LAR].Kd = 0.03;

//         joint[RHY].Kd = joint[LHY].Kd;
//         joint[RHR].Kd = joint[LHR].Kd;
//         joint[RHP].Kd = joint[LHP].Kd;
//         joint[RKN].Kd = joint[LKN].Kd;
//         joint[RAP].Kd = joint[LAP].Kd;
//         joint[RAR].Kd = joint[LAR].Kd;

// }