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
	void setSpeed(const std_msgs::Float32 msg);

private:
	VictorSPX _motor = {DeviceIDs::motor};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "Victor Percent Output");
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
	float percentOutput = msg.data;
	if (percentOutput < -1.0f)
		percentOutput = -1.0f;
	else if (percentOutput > 1.0f)
		percentOutput = 1.0f;

	_motor.Set(ControlMode::PercentOutput, percentOutput);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}