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

//#define PI 3.1415927
//#define DIAMETER 0.2		// should be updated to non-tread diameter of wheels
//#define GEAR_RATIO 26.9
//#define COUNTS_PER_REVOLUTION 7

//#define MAX_SPEED 3.0

// double mps2countsPer100ms(double mps)
// {
// 	return mps*0.1*(1.0/(DIAMETER*PI))*GEAR_RATIO*COUNTS_PER_REVOLUTION;
// }

// double countsPer100ms2mps(double countsPer100ms)
// {
// 	return countsPer100ms/(0.1*(1.0/(DIAMETER*PI))*GEAR_RATIO*COUNTS_PER_REVOLUTION);
// }


class Listener
{
public:
	Listener();
	void setPosition(const std_msgs::Float32 msg);
	double getActualPosition();

private:
	TalonSRX _motor = {DeviceIDs::motor};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "TalonPotPosition");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);
	
	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber position_sub = n.subscribe("speed", 1000, &Listener::setPosition, &listener);
	ros::Publisher actualPosition_pub = n.advertise<std_msgs::Float32>("actualPosition", 100);

	std_msgs::Float32 actualPosition_msg;

	

	while (ros::ok())
	{
		actualPosition_msg.data = listener.getActualPosition();
		
		actualPosition_pub.publish(actualPosition_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


void Listener::setPosition(const std_msgs::Float32 msg)
{
	// limit values
	float fullyExtended = msg.data;
	if (fullyExtended < 0)
		fullyExtended = 0;
	else if (fullyExtended > 1)
		fullyExtended = 1;

	_motor.Set(ControlMode::Position, fullyExtended);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}


Listener::Listener()
{
	TalonSRXConfiguration motorProfile;

	ros::Duration(1.0).sleep();    // give time to finish initialization before trying to read position data, otherwise it will fail

	//Threshold for zero-motion for the neutral position.
	motorProfile.neutralDeadband = 0.01;
			
	//Peak Speed Config
	motorProfile.peakOutputForward = 1.0;
	motorProfile.peakOutputReverse = -1.0;
	
	//Ramp Config
	motorProfile.closedloopRamp = 1.5f;
			
	//PID Config
	//motorProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	motorProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::Analog;
	motorProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	motorProfile.slot0.kP                       = 10.0f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	motorProfile.slot0.kI                       = 0.001f; //Integral Constant.     Controls the steady-state error correction.
	motorProfile.slot0.kD                       = 0.1f; //Derivative Constant.   Controls error oscillation.
	motorProfile.slot0.kF                       = 0.0f; //Feed Forward Constant. (IDK what this does)
	motorProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	motorProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. (IDK what this does)
	motorProfile.slot0.allowableClosedloopError = 0.001;   //If the total error-value is less than this value, the error is automatically set to zero.
	motorProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	motorProfile.slot0.closedLoopPeriod         = 100;   //Samples per second (?) (IDK what this is)

	_motor.ConfigAllSettings(motorProfile);

	_motor.SetNeutralMode(NeutralMode::Brake);
	_motor.SetInverted(false);
	_motor.SetSensorPhase(false);
}

// reads speed of motor shaft and returns in meters/sec
double Listener::getActualPosition()
{
	return _motor.GetSelectedSensorPosition();
}