#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "DeviceIDs.h"

#include "ros/ros.h"
#include "std_msgs/Int64.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "std_msgs/Float64.h"

using namespace std;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;
//using namespace ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice;

const int MAX_RPM = 2000;

class Listener
{
public:
	Listener();
	void setRPM(const std_msgs::Float64 msg);
	void setSP(const std_msgs::Int64 msg);
	void setIntake(const std_msgs::Float64 msg);
	void setCommencement(const std_msgs::Float64 msg);
	void updateLinAct();

	TalonSRX _comArm = {DeviceIDs::commencementArm};
	TalonSRX _linAct = {DeviceIDs::launcherAngle};
	TalonSRX _intakeLeft = {DeviceIDs::intakeLeft};
	TalonSRX _intakeRight = {DeviceIDs::intakeRight};

	float _rpmSetpoint = 0.0;

	PIDController _SPController;
	int _lastSP = 0;
};


int main (int argc, char **argv)
{
    ctre::phoenix::platform::can::SetCANInterface("can1");
    Listener listener;

	ros::Subscriber set_RPM_sub = n.subscribe("set_RPM", 1000, &Listener::setRPM, &listener);
	ros::Subscriber set_intake_sub = n.subscribe("set_intake", 1000, &Listener::setIntake, &listener);
	ros::Subscriber set_commencement_sub = n.subscribe("set_commencement", 1000, &Listener::setCommencement, &listener);

	ros::Publisher pos_pub = n.advertise<std_msgs::Float64>("PotPos", 1000);
	ros::Publisher sp_pub = n.advertise<std_msgs::Float64>("Pot_setpoint", 1000);

	std_msgs::Float64 pos_msg;
	std_msgs::Float64 sp_msg;

	listener._linAct.SetSelectedSensorPosition(0);
	listener._linAct.Set(ControlMode::Position, 0);
	listener._lastSP = 0;
	listener._ActSPController.setKP(1);
	listener._ActSPController.setMaxOut(100);

	while (ros::ok())
	{
		RPM_msg.data = Conversions::toRpm( listener._linAct.GetSelectedSensorVelocity() );

		RPM_pub.publish(top_RPM_msg);

		pos_msg.data = listener._linAct.GetSelectedSensorPosition() / 4096.0 / 100.0 / 85.0 * 42.0 * 360.0 + 34.5;
		pos_pub.publish(pos_msg);

		listener.updatelinAct();
		sp_msg.data = listener._lastSP / 4096.0 / 100.0 / 85.0 * 42.0 * 360.0 + 34.5;
		sp_pub.publish(sp_msg);

		ctre::phoenix::unmanaged::FeedEnable(100); // feed watchdog

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


Listener::Listener()
{
	// // ============================== Top Wheel ==============================
	// TalonSRXConfiguration topProfile;

	// //Threshold for zero-motion for the neutral position.
	// topProfile.neutralDeadband = 0.01;
			
	// //Peak Speed Config
	// topProfile.peakOutputForward = 1.0;
	// topProfile.peakOutputReverse = -1.0;
	
	// //Ramp Config
	// topProfile.closedloopRamp = 1.5f;
			
	// //PID Config
	// topProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::Analog;
	// topProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	// //PID Constants
	// topProfile.slot0.kP                       = 0.025f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	// topProfile.slot0.kI                       = 0.0165f; //Integral Constant.     Controls the steady-state error correction.
	// topProfile.slot0.kD                       = 0.016f; //Derivative Constant.   Controls error oscillation.
	// topProfile.slot0.kF                       = 0.0084f; //Feed Forward Constant. (IDK what this does)
	// topProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	// topProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. (IDK what this does)
	// topProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	// topProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	// topProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	// TalonSRXPIDSetConfiguration profile;

	// _linAct.ConfigAllSettings(topProfile);

	// _linAct.SetNeutralMode(NeutralMode::Brake);
	// _linAct.SetInverted(false);
	// _linAct.SetSensorPhase(false);

	// // ============================== Bottom Wheel ==============================

	// TalonSRXConfiguration bottomProfile;

	// //Threshold for zero-motion for the neutral position.
	// bottomProfile.neutralDeadband = 0.01;
			
	// //Peak Speed Config
	// bottomProfile.peakOutputForward = 1.0;
	// bottomProfile.peakOutputReverse = -1.0;
			
	// //Ramp Config
	// bottomProfile.closedloopRamp = 1.5f;
			
	// //PID Config
	// bottomProfile.primaryPID.selectedFeedbackSensor = FeedbackDevice::QuadEncoder;
	// bottomProfile.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	// //PID Constants
	// bottomProfile.slot0.kP                       = 0.025f; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	// bottomProfile.slot0.kI                       = 0.0165f; //Integral Constant.     Controls the steady-state error correction.
	// bottomProfile.slot0.kD                       = 0.016f; //Derivative Constant.   Controls error oscillation.
	// bottomProfile.slot0.kF                       = 0.0084; //Feed Forward Constant. For velocity
	// bottomProfile.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	// bottomProfile.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	// bottomProfile.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	// bottomProfile.slot0.closedLoopPeakOutput     = 1.0f; //Peak output for the PID Controller.
	// bottomProfile.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	// _bottomWheel.ConfigAllSettings(bottomProfile);

	// _bottomWheel.SetNeutralMode(NeutralMode::Brake);
	// _bottomWheel.SetInverted(false);
	// _bottomWheel.SetSensorPhase(true);


	// // =========================== Commencement Arm =============================
	 _comArm.SetInverted(true);

	// ============================== Angle Motor ===============================
	TalonSRXConfiguration linActProf;

	//Threshold for zero-motion for the neutral position.
	linActProf.neutralDeadband = 0.01;
			
	//Peak Speed Config
	linActProf.peakOutputForward = 0.5;
	linActProf.peakOutputReverse = -0.5;
			
	//Ramp Config
	linActProf.closedloopRamp = 1.5f;
			
	//PID Config
	linActProf.primaryPID.selectedFeedbackSensor = FeedbackDevice::Analog;
	linActProf.primaryPID.selectedFeedbackCoefficient = 1.0f;//0.25f;// 0.328293f;

	//PID Constants
	linActProf.slot0.kP                       = 0.01; //0.01f; //Propotional Constant.  Controls the speed of error correction.
	linActProf.slot0.kI                       = 0.006; //Integral Constant.     Controls the steady-state error correction.
	linActProf.slot0.kD                       = 0.06; //Derivative Constant.   Controls error oscillation.
	linActProf.slot0.kF                       = 0.0; //Feed Forward Constant. For velocity
	linActProf.slot0.integralZone             = 100000;   //Maximum value for the integral error accumulator. Automatically cleared when exceeded.
	linActProf.slot0.maxIntegralAccumulator   = 10000;   //Maximum value for the integral error accumulator. Biggest Error for I
	linActProf.slot0.allowableClosedloopError = 217;   //If the total error-value is less than this value, the error is automatically set to zero.
	linActProf.slot0.closedLoopPeakOutput     = 0.3f; //Peak output for the PID Controller.
	linActProf.slot0.closedLoopPeriod         = 500;   //Samples per second (?) (IDK what this is)

	_linAct.ConfigAllSettings(linActProf);

	_linAct.SetNeutralMode(NeutralMode::Brake);
	_linAct.SetInverted(true);
	_linAct.SetSensorPhase(true);
}


void Listener::setRPM(const std_msgs::Float64 msg)
{
	float rpm = msg.data;
	if (rpm > 0.01)
	{
		if (rpm > MAX_RPM)
			rpm = MAX_RPM;
		
		_rpmSetpoint = rpm;
		_linAct.Set(ControlMode::Velocity, Conversions::fromRpm(rpm - 100));
	}
	else
	{
		_rpmSetpoint = 0.0;
		_linAct.Set(ControlMode::PercentOutput, 0.0);
	}
}

void Listener::SetSP(const std_msgs::Int64 msg)
{
	if (msg.data < 0)
		_linAct.SetSelectedSensorPosition(0);
	else
		_linActSPController.setSetpoint(msg.data);
}

void Listener::updatelinAct()
{
	_lastSP = _lastSP + _linActSPController.calcOutput(_lastSP);
	_linAct.Set(ControlMode::Position, _lastSP);
}

void Listener::setIntake(const std_msgs::Float64 msg)
{
	_intakeLeft.Set(ControlMode::PercentOutput, -1 * msg.data);
	_intakeRight.Set(ControlMode::PercentOutput, msg.data);
}

void Listener::setCommencement(const std_msgs::Float64 msg)
{
	#ifndef COMMENCEMENT
		return;
	#endif
	_comArm.Set(ControlMode::PercentOutput, msg.data);
}




}