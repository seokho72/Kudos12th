#ifndef KUDOS_H_
#define KUDOS_H_

//* Basic header, Library
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI 

#define tasktime    0.001

//* Command flag mode
typedef enum {
    NONE_ACT,
    NO1,
    RETURN_POSE

} _COMMAND_FLAG;


//* Control Mode
typedef enum {
    WRITEMODE

} _CONTROL_MODE;


//* struct for Joints and End Points variables
typedef struct Joint {
    //* Current information

    float currentAngle; // [rad]

} JOINT;


class Kudos {
public:

    Kudos();
    Kudos(const Kudos& orig);
    virtual ~Kudos();

    //***Variables***//


    double Link1 = 0.1;
    double Link2 = 0.12;


    struct End_point {
      double x;
      double y;
    }; // x,y point in, struct 함수 : 구조체, 안에 변수형 선언(다른 타입 가능)


    struct Joint {
      double TH1;
      double TH2;
    };


    //* Control Mode and Command Flag
    unsigned int ControlMode;
    unsigned int CommandFlag;

    //* enum

    enum JointNumber { // Kubot
        LK1 = 0, LK2
        
    };

    //***Functions***//

    // VectorXd init_pose(int param_1, VectorXd present, VectorXd real);

    bool Move_current;
    float refAngle[2];
    float walkReadyAngle[2];
    float write_ready_joint[2]; // Now nDoF = 2

    void ReturnPose(float return_time);
    void NO1(float readytime);
    double cosWave(double amp, double period, double time, double int_pos);
    void Compute_IK(double EP_x, double EP_y);

    // bool wave_flag;

    //* Functions related to Walking, are in CRobot_Walking.cpp

private:

    //***Variables***//    

    //***Functions***//

};
#endif