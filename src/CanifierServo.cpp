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
	float pulse = 0;
	if (pos < -1.0f)
		pos = -1.0f;
	else if (pos > 1.0f)
		pos = 1.0f;
	int flag = 0;

	if (flag == 0){
		_canifer.SetGeneralOutput(CANifier::GeneralPin::SPI_CLK_PWM0P, false, true);
		flag = 1;
	}

	_canifer.SetGeneralOutput(CANifier::GeneralPin::QUAD_A, pos > 0.5 ? true : false, true);
	
	pulse = LinearInterpolation::Calculate(pos, -1, 1000, 1, 2000);
	pulse = pulse/4200;

	_canifer.SetPWMOutput(0, pulse);
	

	
	_canifer.EnablePWMOutput(0, true);

	ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog
}
