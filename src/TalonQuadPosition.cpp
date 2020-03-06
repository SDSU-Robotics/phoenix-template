#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

#include "ros/ros.h"
#include "std_msgs/Float32.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

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

	ros::Subscriber speed_sub = n.subscribe("speed", 1000, &Listener::setAngle, &listener);

	ros::spin();

	return 0;
}


void Listener::setAngle(const std_msgs::Float32 msg)
{
	// limit values
	float percentOutput = msg.data;
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motor.Set(ControlMode::PercentOutput, percentOutput);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}

Listener::Listener()
{
    TalonSRXConfiguration angleProfile;

	//Threshold for zero-motion for the neutral position.
	angleProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	angleProfile.peakOutputForward = 0.5;
	angleProfile.peakOutputReverse = -0.5;
			
	//Ramp Config
	angleProfile.closedloopRamp = 1.5f;
			
	//PID Config
	angleProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	angleProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	angleProfile.slot0.kP                       = 0.01; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	angleProfile.slot0.kI                       = 0.006; //Integral Constant.     Controls the steady-state error correction.
	angleProfile.slot0.kD                       = 0.06; //Derivative Constant.   Controls error oscillation.
	angleProfile.slot0.kF                       = 0.0; //Feed Forward Constant. For velocity
	angleProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	angleProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	angleProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero
	angleProfile.slot0.closedLoopPeakOutput     = 0.3f; //Peak output for the PID Controller.
	angleProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	_motor.ConfigAllSettings(angleProfile);

	_motor.SetNeutralMode(NeutralMode::Brake);
	_motor.SetInverted(true);
	_motor.SetSensorPhase(true);
}