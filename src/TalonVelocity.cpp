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

#define PI 3.1415927
#define DIAMETER 0.2
#define GEAR_RATIO 26.9
#define COUNTS_PER_REVOLUTION 7

#define MAX_SPEED 1.0

double mps2counts(double mps)
{
	return mps*0.1*(1.0/(DIAMETER*PI))*GEAR_RATIO*COUNTS_PER_REVOLUTION;
}


class Listener
{
public:
	Listener();
	void setSpeed(const std_msgs::Float32 msg);

private:
	TalonSRX _motor = {DeviceIDs::motor};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "TalonVelocity");
	ros::NodeHandle n;
	
	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber speed_sub = n.subscribe("speed", 1000, &Listener::setSpeed, &listener);

	ros::spin();

	return 0;
}


void Listener::setSpeed(const std_msgs::Float32 msg)
{
	// limit values
	float mps = msg.data;
	if (mps < -MAX_SPEED)
		mps = -MAX_SPEED;
	else if (mps > MAX_SPEED)
		mps = MAX_SPEED;

	cout << mps2counts(mps) << endl;
	cout << _motor.GetSelectedSensorVelocity() << endl;
	_motor.Set(ControlMode::Velocity, mps2counts(mps));

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}


Listener::Listener()
{
	TalonSRXConfiguration motorProfile;

	//Threshold for zero-motion for the neutral position.
	motorProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	motorProfile.peakOutputForward = 1.0;
	motorProfile.peakOutputReverse = -1.0;
	
	//Ramp Config
	motorProfile.closedloopRamp = 1.5f;
			
	//PID Config
	motorProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	motorProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	motorProfile.slot0.kP                       = 100.0f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	motorProfile.slot0.kI                       = 0.0f; //Integral Constant.     Controls the steady-state error correction.
	motorProfile.slot0.kD                       = 0.0f; //Derivative Constant.   Controls error oscillation.
	motorProfile.slot0.kF                       = 0.0f; //Feed Forward Constant. (IDK what this does)
	motorProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	motorProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. (IDK what this does)
	motorProfile.slot0.allowableClosedloopError = 0.0;   //If the total error-value is less than this value, the error is automatically set to zero.
	motorProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	motorProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	_motor.ConfigAllSettings(motorProfile);

	_motor.SetNeutralMode(NeutralMode::Brake);
	_motor.SetInverted(false);
	_motor.SetSensorPhase(false);

}

