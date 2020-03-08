#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

#include <iostream>

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


#define COUNTS_PER_REVOLUTION 7

double deg2counts(double deg)
{
	return deg/360.0 * COUNTS_PER_REVOLUTION;
}

class Listener
{
public:
    Listener();
	void setAngle(const std_msgs::Float32 msg);

private:
	TalonSRX _motor = {DeviceIDs::motor};
};


int main (int argc, char **argv)
{

	ros::init(argc, argv, "TalonQuadPosition");
	ros::NodeHandle n;
	
	ctre::phoenix::platform::can::SetCANInterface("can0");
	
	Listener listener;
	
	ros::Subscriber angle_sub = n.subscribe("angle", 1000, &Listener::setAngle, &listener);

	ros::spin();

	return 0;
}


void Listener::setAngle(const std_msgs::Float32 msg)
{
	float angle = msg.data;

	_motor.Set(ControlMode::Position, deg2counts(angle));

	cout << _motor.GetSelectedSensorPosition() << endl;

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

Listener::Listener()
{
	ros::Duration(1.0).sleep();

    TalonSRXConfiguration motorProfile;

	//Threshold for zero-motion for the neutral position.
	motorProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	motorProfile.peakOutputForward = 0.5;
	motorProfile.peakOutputReverse = -0.5;
			
	//Ramp Config
	motorProfile.closedloopRamp = 1.5f;
			
	//PID Config
	motorProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	motorProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	motorProfile.slot0.kP                       = 100; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	motorProfile.slot0.kI                       = 0.006; //Integral Constant.     Controls the steady-state error correction.
	motorProfile.slot0.kD                       = 0.06; //Derivative Constant.   Controls error oscillation.
	motorProfile.slot0.kF                       = 0.0; //Feed Forward Constant. For velocity
	motorProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	motorProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	motorProfile.slot0.allowableClosedloopError = 0;   //If the total error-value is less than this value, the error is automatically set to zero
	motorProfile.slot0.closedLoopPeakOutput     = 0.3f; //Peak output for the PID Controller.
	motorProfile.slot0.closedLoopPeriod         = 200;   //Samples per second (?) (IDK what this is)

	_motor.ConfigAllSettings(motorProfile);

	_motor.SetNeutralMode(NeutralMode::Brake);
	_motor.SetInverted(true);
	_motor.SetSensorPhase(true);
	_motor.SetSelectedSensorPosition(0);
	_motor.Set(ControlMode::Position,(0));
}
