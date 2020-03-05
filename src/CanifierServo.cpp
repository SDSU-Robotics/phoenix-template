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
// Sterling Berg



class Listener
{
public:
	void setPosition(const std_msgs::Float32 msg);

private:
	CANifier _canifer = {DeviceIDs::canifier};
};


int main (int argc, char **argv)
{
	ros::init(argc, argv, "CanifierServo");
	ros::NodeHandle n;
	
	ctre::phoenix::platform::can::SetCANInterface("can0");

	Listener listener;

	ros::Subscriber speed_sub = n.subscribe("speed", 1000, &Listener::setPosition, &listener);

	ros::spin();

	return 0;
}


void Listener::setPosition(const std_msgs::Float32 msg)
{
	// limit values
	float pos = msg.data;
	if (pos < 0.0f)
		pos = 0.0f;
	else if (pos > 1.0f)
		pos = 1.0f;

	_canifer.SetGeneralOutput(CANifier::GeneralPin::SDA, pos > 0.5 ? true : false, true);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
