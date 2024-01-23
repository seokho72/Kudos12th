#include "Kudos.h"
// #include "kinematics.h"

using namespace std;

Kudos::Kudos()  // Kudos 클래스의 기본 생성자
{
}  
Kudos::Kudos(const Kudos &orig)  // Kudos 클래스의 복사 생성자
{
}
Kudos::~Kudos()  // Kudos 클래스의 소멸자
{
}

double Kudos::cosWave(double amp, double period, double time, double int_pos)
{
    return (amp / 2) * (1 - cos(PI / period * time)) + int_pos;
}

void Kudos::ReturnPose(float return_time)
{
		    static float re_time = 0;
            static struct End_point target;
	      static struct End_point EP_goal;
	      static float init_joint_angle[2]; // Now nDoF = 13

			//EndPoint position 설정
        EP_goal.x = 0.22;
        EP_goal.y = 0.0;

			//IK 함수
	    //EndPoint를 원하는 위치로 보내기 위한 링크의 각도가 어떻게 바뀌어야 하는가를 계산하는 과정
        Compute_IK(EP_goal.x, EP_goal.y);

        if (re_time == 0 ) {
            for (int j = 0; j < 2; j++) {
                init_joint_angle[j] = refAngle[j];
            }
        }

        if (re_time <= return_time)
        {
            for (int j = 0; j < 2; j++) {
                refAngle[j] = cosWave(write_ready_joint[j] - init_joint_angle[j], return_time, re_time, init_joint_angle[j]);
            }
            re_time += tasktime;
        }
        else
        {
            re_time=0;
            Move_current = false;
        }
}

void Kudos::NO1(float readytime){

	    static float time = 0;
        static struct End_point target;
        static struct End_point EP_goal;
        static float currentAngle[2]; // Now nDoF = 13

        EP_goal.x = 0.1;
        EP_goal.y = 0.0;
        Compute_IK(EP_goal.x, EP_goal.y);

        if (time == 0 ) {
            for (int j = 0; j < 2; j++) {
                currentAngle[j] = refAngle[j];
            }
        }

        if (time <= readytime)
        {
            for (int j = 0; j < 2; j++) {
                refAngle[j] = cosWave(write_ready_joint[j] - currentAngle[j], readytime, time, currentAngle[j]);
            }
            time += tasktime;
        }
        else
        {
            time=0;
            Move_current = false;
        }
}

void Kudos::Compute_IK(double EP_x, double EP_y) {

  //IK soluttion
  /*
  struct Joint J;
  J.TH1 = atan2(sin(J.TH1), cos(J.TH2));
  J.TH2 = atan2(sqrt((1 - pow(cos(J.TH1), 2))), cos(J.TH2));
  return J;
  */

  double x = EP_x;
  double y = EP_y;
  
	

    double alpha = atan2(y,x); //tan의 역함수 y/x
    double L = sqrt(pow(x,2)+pow(y,2)); //sqrt : 루트, pow(x,n) : x의 n승
    double beta = acos((pow(Link1,2)+pow(Link2,2)-pow(L,2))/(2*Link1*Link2));
    double gamma = atan2(x,y);
    double delta = acos((pow(Link1,2)+pow(L,2)-pow(Link2,2))/(2*Link1*L));

    double th2 = PI - beta;
    double th1 = (PI)/2 - gamma - delta;
    

  printf("%f , %f", th1, th2);

  write_ready_joint[0] = th1;
  write_ready_joint[1] = th2;

}