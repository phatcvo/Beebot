#include <iostream>
#include "remote_control.h"


using namespace std;

ros::Subscriber joy_sub;
ros::Publisher control_pub, control_pub3;
ros::Time last_received_time;
ros::Timer timer;
bool isConnected = false;
bool isResetMode = false;

Remote_Control::Remote_Control(ros::NodeHandle& nh)
{
	Remote_Control::initParam(nh);
	// BUTTON
	m_nCross_control=0;
	m_nJoy_control=0;
	m_nEstop=0;
	m_nNoEstop=0;
	m_nLinearSpeedDown=0;
	m_nLinearSpeedUp=0;
	m_nAngularSpeedDown=0;
	m_nAngularSpeedUp=0;
	m_nAutoMode=0;
	m_nManualMode=0;
	m_nStickButton_L=0;
	m_nStickButton_R=0;
	test_data = 0;

	// AXIS
	m_dLinear_stick=0.0;
	m_dAngular_stick=0.0;
	m_nLinear_cross=0.0;
	m_nAngular_cross=0.0;

	// Control output
	m_dLinear_spd=0.0;
	m_dAngular_spd=0.0;

	// States
	m_binitStart_joy=false;
	m_bEstop=false;
	m_bLinearSpeedDown=false;
	m_bLinearSpeedUp=false;
	m_bAngularSpeedDown=false;
	m_bAngularSpeedUp=false;
	m_bCrossUp=false;
	m_bCrossDown=false;
	m_bCrossLeft=false;
	m_bCrossRight=false;
	m_nMode=STATE::MANUAL;
	m_nControlMode=STATE::JOY;
	m_nLinearStep=0;
	m_nAngularStep=0;

	// Parameters
	m_dmin_Linear = parameters.min_Linear;
	m_dMax_Linear = parameters.Max_Linear;
	m_dmin_Angular = parameters.min_Angular;
	m_dMax_Angular = parameters.Max_Angular;

	m_nControl_ratio = parameters.control_ratio;
	m_dLinear_scale = parameters.init_linear_vel;
	m_dAngular_scale = parameters.init_angular_vel;
	m_dLinear_control = (m_dMax_Linear - m_dmin_Linear)/(double)m_nControl_ratio;
	m_dAngular_control = (m_dMax_Angular - m_dmin_Angular)/(double)m_nControl_ratio;

	m_dLinear_step_scale = parameters.init_linear_step_scale;
	m_dAngular_step_scale = parameters.init_angular_step_scale;
	m_dLinear_step_control = parameters.linear_step_control_size;
	m_dAngular_step_control = parameters.angular_step_control_size;
	
	control_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	printf("********* config done ********\n");

	// Initialize the subscriber in the constructor
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &Remote_Control::joyCallback, this);
}

void Remote_Control::initParam(ros::NodeHandle& n)
{
	n.param<int>("/key_type", parameters.key_type, 0);
	n.param<double>("/reset_mode_timeout", parameters.reset_mode_timeout, 10.0);
	n.param<double>("/connection_timeout", parameters.connection_timeout, 0.3);
	n.param<double>("/min_Linear", parameters.min_Linear, 0.2);
	n.param<double>("/Max_Linear", parameters.Max_Linear, 1.4);
	n.param<double>("/min_Angular", parameters.min_Angular, 0.3);
	n.param<double>("/Max_Angular", parameters.Max_Angular, 1.0);
	n.param<int>("/control_ratio", parameters.control_ratio, 8);
	n.param<double>("/init_linear_vel", parameters.init_linear_vel, 0.1);
	n.param<double>("/init_angular_vel", parameters.init_angular_vel, 0.1);
	n.param<double>("/init_linear_step_scale", parameters.init_linear_step_scale, 0.2);
	n.param<double>("/init_angular_step_scale", parameters.init_angular_step_scale, 0.2);
	n.param<double>("/linear_step_control_size", parameters.linear_step_control_size, 0.1);
	n.param<double>("/angular_step_control_size", parameters.angular_step_control_size, 0.1);
	
	ROS_INFO_STREAM("=== REMOTE CONTROLLER PARAM ===");
	ROS_INFO_STREAM("key_type: "<< parameters.key_type);
	ROS_INFO_STREAM("reset_mode_timeout: "<< parameters.reset_mode_timeout);
	ROS_INFO_STREAM("connection_timeout: "<< parameters.connection_timeout);
	ROS_INFO_STREAM("min_Linear: "<< parameters.min_Linear);
	ROS_INFO_STREAM("Max_Linear: "<< parameters.Max_Linear);
	ROS_INFO_STREAM("min_Angular: "<< parameters.min_Angular);
	ROS_INFO_STREAM("Max_Angular: "<< parameters.Max_Angular);
	ROS_INFO_STREAM("control_ratio: "<< parameters.control_ratio);
	ROS_INFO_STREAM("init_linear_vel: "<< parameters.init_linear_vel);
	ROS_INFO_STREAM("init_angular_vel: "<< parameters.init_angular_vel);
	ROS_INFO_STREAM("linear_step_control_size: "<< parameters.linear_step_control_size);
	ROS_INFO_STREAM("angular_step_control_size: "<< parameters.angular_step_control_size);
}

void Remote_Control::PS4(const sensor_msgs::Joy::ConstPtr& joy){
    m_nNoEstop = joy->buttons[PS4_BTN::A];
	m_nEstop = joy->buttons[PS4_BTN::B];
	m_nCross_control = joy->buttons[PS4_BTN::X];
	m_nJoy_control = joy->buttons[PS4_BTN::Y];
	m_nLinearSpeedDown = joy->buttons[PS4_BTN::LB];
	m_nLinearSpeedUp = joy->buttons[PS4_BTN::RB];
	m_nAngularSpeedDown = joy->buttons[PS4_BTN::LT];
	m_nAngularSpeedUp = joy->buttons[PS4_BTN::RT];
	m_nAutoBack = joy->buttons[PS4_BTN::back];
	m_nAutoMode = joy->buttons[PS4_BTN::start];
	m_nManualMode = joy->buttons[PS4_BTN::mode];
	m_nStickButton_L = joy->buttons[PS4_BTN::Button_stick_L];
	m_nStickButton_R = joy->buttons[PS4_BTN::Button_stick_R];

	m_dAngular_stick = joy->axes[PS4_AXIS::Axis_LR1];
	m_dLinear_stick = joy->axes[PS4_AXIS::Axis_UD1];
	m_dAngular_stick2 = joy->axes[PS4_AXIS::Axis_LR2];
	m_dLinear_stick2 = joy->axes[PS4_AXIS::Axis_UD2];
	m_nAngular_cross = joy->axes[PS4_AXIS::Cross_key_LR];
	m_nLinear_cross = joy->axes[PS4_AXIS::Cross_key_UD];
}

void Remote_Control::XBOX(const sensor_msgs::Joy::ConstPtr& joy){
    m_nNoEstop = joy->buttons[XBOX_BTN::A];
	m_nEstop = joy->buttons[XBOX_BTN::B];
	m_nCross_control = joy->buttons[XBOX_BTN::X];
	m_nJoy_control = joy->buttons[XBOX_BTN::Y];
	m_nLinearSpeedDown = joy->buttons[XBOX_BTN::LB];
	m_nLinearSpeedUp = joy->buttons[XBOX_BTN::RB];
	
	m_nAutoBack = joy->buttons[XBOX_BTN::back];
	m_nAutoMode = joy->buttons[XBOX_BTN::start];
	m_nManualMode = joy->buttons[XBOX_BTN::mode];
	m_nStickButton_L = joy->buttons[XBOX_BTN::Button_stick_L];
	m_nStickButton_R = joy->buttons[XBOX_BTN::Button_stick_R];

    m_nAngularSpeedDown = joy->buttons[XBOX_AXIS::LT];
	m_nAngularSpeedUp = joy->buttons[XBOX_AXIS::RT];
	m_dAngular_stick = joy->axes[XBOX_AXIS::Axis_LR1];
	m_dLinear_stick = joy->axes[XBOX_AXIS::Axis_UD1];
	m_dAngular_stick2 = joy->axes[XBOX_AXIS::Axis_LR2];
	m_dLinear_stick2 = joy->axes[XBOX_AXIS::Axis_UD2];
	m_nAngular_cross = joy->axes[XBOX_AXIS::Cross_key_LR];
	m_nLinear_cross = joy->axes[XBOX_AXIS::Cross_key_UD];
}

void Remote_Control::LOGITECH(const sensor_msgs::Joy::ConstPtr& joy){
    m_nNoEstop = joy->buttons[LOGITECH_BTN::A];
	m_nEstop = joy->buttons[LOGITECH_BTN::B];
	m_nCross_control = joy->buttons[LOGITECH_BTN::X];
	m_nJoy_control = joy->buttons[LOGITECH_BTN::Y];
	m_nLinearSpeedDown = joy->buttons[LOGITECH_BTN::LB];
	m_nLinearSpeedUp = joy->buttons[LOGITECH_BTN::RB];
	m_nAngularSpeedDown = joy->buttons[LOGITECH_BTN::LT];
	m_nAngularSpeedUp = joy->buttons[LOGITECH_BTN::RT];
	m_nAutoBack = joy->buttons[LOGITECH_BTN::back];
	m_nAutoMode = joy->buttons[LOGITECH_BTN::start];
	m_nManualMode = joy->buttons[LOGITECH_BTN::mode];
	m_nStickButton_L = joy->buttons[LOGITECH_BTN::Button_stick_L];
	m_nStickButton_R = joy->buttons[LOGITECH_BTN::Button_stick_R];

	m_dAngular_stick = joy->axes[LOGITECH_AXIS::Axis_LR1];
	m_dLinear_stick = joy->axes[LOGITECH_AXIS::Axis_UD1];
	m_dAngular_stick2 = joy->axes[LOGITECH_AXIS::Axis_LR2];
	m_dLinear_stick2 = joy->axes[LOGITECH_AXIS::Axis_UD2];
	m_nAngular_cross = joy->axes[LOGITECH_AXIS::Cross_key_LR];
	m_nLinear_cross = joy->axes[LOGITECH_AXIS::Cross_key_UD];
}


void Remote_Control::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(parameters.key_type == TYPE::PS4) {
        PS4(joy);
    } else if(parameters.key_type == TYPE::XBOX) {
        XBOX(joy);
    } else if(parameters.key_type == TYPE::LOGITECH) {
        LOGITECH(joy);
    } else {
        ROS_ERROR("Invalid key type");
    }


	// Joy-controller Start
	if(!m_binitStart_joy) {
		if(m_nManualMode==STATE::ON) {
			m_binitStart_joy=true;
			cout<<"*****START PLAYING*****"<<endl;
		}
	}
	
	// Control
	if(m_binitStart_joy) {
		if(!E_stop()) {
			printf(">>>>>>>>>>> [B-->A] E-STOP <<<<<<<<<< \n");
			if(MODE()) {
				printf(_MAGENTA_ "[mode ] MODE: Manual\n" _RESET_);
				CONTROL_MODE();
				SPEED_CONTROL();
				CALC_SPEED();
			}
			else {
                printf(_GREEN_ "[start]  MODE: Auto\n" _RESET_);
            }
		}

		geometry_msgs::Twist output;
		output.linear.x = m_dLinear_spd;
		output.angular.z = m_dAngular_spd;
		output.linear.z = m_nMode;
		printf(_YELLOW_ "ControlCommand sent: %lf, %lf, %d\n" _RESET_, m_dLinear_spd, m_dAngular_spd, m_nMode);
		control_pub.publish(output);
	}

	printf("---------------------------------\n\n");
	last_received_time = ros::Time::now();
	isConnected = true;
	isResetMode = false;
	timer = nh_.createTimer(ros::Duration(0.1), &Remote_Control::checkConnection, this);
}

bool Remote_Control::E_stop(void)
{
	printf("m_nEstop: %d\n", m_nEstop);
	if(m_nEstop==STATE::ON) {
		m_bEstop=true;
		m_nMode=STATE::MANUAL;
		m_nLinearStep=0;
		m_nAngularStep=0;
		m_dLinear_spd=0.0;
		m_dAngular_spd=0.0;
		ROS_ERROR("E-STOP !!!!!!!!!!");
	}
	else if(m_bEstop && m_nNoEstop==STATE::ON) {
		m_bEstop=false;
		ROS_WARN("Release E-stop !!");
	}

	if(m_bEstop) return true;
	return false;
}

bool Remote_Control::MODE(void)
{
	if(m_nMode==STATE::MANUAL && m_nAutoMode==STATE::ON) {
		m_nMode=STATE::AUTO;
		m_dLinear_spd=0;
		m_dAngular_spd=0;
		m_nLinearStep=0;
		m_nAngularStep=0;
		ROS_INFO("MODE: Auto");
	}
	else if(m_nMode==STATE::AUTO && m_nManualMode==STATE::ON) {
		m_nMode=STATE::MANUAL;
		ROS_INFO("MODE: Manual");
	}

	if(m_nMode==STATE::MANUAL) return true;
	return false;
}

void Remote_Control::CONTROL_MODE(void)
{
	if(m_nControlMode==STATE::JOY && m_nCross_control==STATE::ON) {
		m_nControlMode=STATE::CROSS;
		m_dLinear_spd=0;
		m_dAngular_spd=0;
		ROS_INFO("CONTROL MODE: CROSS");
	}
	else if(m_nControlMode==STATE::CROSS && m_nJoy_control==STATE::ON) {
		m_nControlMode=STATE::JOY;
		m_dLinear_spd=0;
		m_dAngular_spd=0;
		m_nLinearStep=0;
		m_nAngularStep=0;
		ROS_INFO("CONTROL MODE: JOY");
	}
}

void Remote_Control::SPEED_CONTROL(void)
{
	bool bprint=true;
	if(m_nControlMode==STATE::JOY) {
		SUB_SPEED_CONTROL(bprint, m_bLinearSpeedUp, m_nLinearSpeedUp, STATE::ON, STATE::OFF, m_dLinear_scale, m_dLinear_control, m_dmin_Linear, m_dMax_Linear);
		SUB_SPEED_CONTROL(bprint, m_bAngularSpeedUp, m_nAngularSpeedUp, STATE::ON, STATE::OFF, m_dAngular_scale, m_dAngular_control, m_dmin_Angular, m_dMax_Angular);
		SUB_SPEED_CONTROL(bprint, m_bLinearSpeedDown, m_nLinearSpeedDown, STATE::ON, STATE::OFF, m_dLinear_scale, -m_dLinear_control, m_dmin_Linear, m_dMax_Linear);
		SUB_SPEED_CONTROL(bprint, m_bAngularSpeedDown, m_nAngularSpeedDown, STATE::ON, STATE::OFF, m_dAngular_scale, -m_dAngular_control, m_dmin_Angular, m_dMax_Angular);

		if(bprint) {
			cout<<"[LB/RB] Linear Scale of Joystick Control: "<<m_dLinear_scale;
			if(m_dLinear_scale==m_dMax_Linear) cout<<" (Max)";
			else if(m_dLinear_scale==m_dmin_Linear) cout<<" (min)";
			cout<<"\n[LT/RT] Angular Scale of Joystick Control: "<<m_dAngular_scale;
			if(m_dAngular_scale==m_dMax_Angular) cout<<" (Max)\n";
			else if(m_dAngular_scale==m_dmin_Angular) cout<<" (min)\n";
			else cout<<endl;
		}
	}
	else if(m_nControlMode==STATE::CROSS) {
		double dMax_Linear_step=m_dMax_Linear/4;
		double dMax_Angular_step=m_dMax_Angular/4;
		SUB_SPEED_CONTROL(bprint, m_bLinearSpeedUp, m_nLinearSpeedUp, STATE::ON, STATE::OFF, m_dLinear_step_scale, m_dLinear_step_control, m_dLinear_step_control, dMax_Linear_step);
		SUB_SPEED_CONTROL(bprint, m_bAngularSpeedUp, m_nAngularSpeedUp, STATE::ON, STATE::OFF, m_dAngular_step_scale, m_dAngular_step_control, m_dAngular_step_control, dMax_Angular_step);
		SUB_SPEED_CONTROL(bprint, m_bLinearSpeedDown, m_nLinearSpeedDown, STATE::ON, STATE::OFF, m_dLinear_step_scale, -m_dLinear_step_control, m_dLinear_step_control, dMax_Linear_step);
		SUB_SPEED_CONTROL(bprint, m_bAngularSpeedDown, m_nAngularSpeedDown, STATE::ON, STATE::OFF, m_dAngular_step_scale, -m_dAngular_step_control, m_dAngular_step_control, dMax_Angular_step);

		if(bprint) {
			cout<<"[LB/RB] Linear Step of Cross Control : "<<m_dLinear_step_scale;
			if(m_dLinear_step_scale==dMax_Linear_step) cout<<" (Max)";
			else if(m_dLinear_step_scale==m_dAngular_step_control) cout<<" (min)";
			cout<<"\n[LT/RT] Angular Step of Cross Control : "<<m_dAngular_step_scale;
			if(m_dAngular_step_scale==dMax_Angular_step) cout<<" (Max)\n";
			else if(m_dAngular_step_scale==m_dAngular_step_control) cout<<" (min)\n";
			else cout<<endl;
		}
	}
}

void Remote_Control::CALC_SPEED(void)
{
	if(m_nControlMode==STATE::JOY) {
		m_dLinear_spd=m_dLinear_scale*m_dLinear_stick;
		m_dAngular_spd=m_dAngular_scale*m_dAngular_stick2;
		printf(_GREEN_ "[Y-->X] CONTROL MODE: JOY\n" _RESET_);
		cout<<"Linear spd : "<<m_dLinear_spd<<" (m/s)"<<endl;
		cout<<"Angular spd: "<<m_dAngular_spd<<" (rad/s)"<<endl;
	} else if (m_nControlMode==STATE::CROSS) {
		printf(_CYAN_ "[X-->Y] CONTROL MODE: CROSS\n" _RESET_);
		bool bprint=false;
		double dLinearStep=(double)m_nLinearStep;
		double dAngularStep=(double)m_nAngularStep;
		SUB_SPEED_CONTROL(bprint, m_bCrossUp, m_nLinear_cross, STATE::UP, STATE::OFF, dLinearStep, 1, -floor(m_dMax_Linear/m_dLinear_step_scale), floor(m_dMax_Linear/m_dLinear_step_scale));
		SUB_SPEED_CONTROL(bprint, m_bCrossDown, m_nLinear_cross, STATE::DOWN, STATE::OFF, dLinearStep, -1, -floor(m_dMax_Linear/m_dLinear_step_scale), floor(m_dMax_Linear/m_dLinear_step_scale));
		SUB_SPEED_CONTROL(bprint, m_bCrossLeft, m_nAngular_cross, STATE::LEFT, STATE::OFF, dAngularStep, 1, -floor(m_dMax_Angular/m_dAngular_step_scale), floor(m_dMax_Angular/m_dAngular_step_scale));
		SUB_SPEED_CONTROL(bprint, m_bCrossRight, m_nAngular_cross, STATE::RIGHT, STATE::OFF, dAngularStep, -1, -floor(m_dMax_Angular/m_dAngular_step_scale), floor(m_dMax_Angular/m_dAngular_step_scale));
		m_nLinearStep=(int)dLinearStep;
		m_nAngularStep=(int)dAngularStep;
		m_dLinear_spd=m_dLinear_step_scale*m_nLinearStep;
		m_dAngular_spd=m_dAngular_step_scale*m_nAngularStep;
		if(bprint) {
			cout<<"Linear spd : "<<m_dLinear_spd<<" (m/s)"<<endl;
			cout<<"Angular spd: "<<m_dAngular_spd<<" (rad/s)"<<endl;
		}
	} else {
		m_dLinear_spd=0;
		m_dAngular_spd=0;
		ROS_ERROR("ERROR: Control Mode is invalid");
	}
}

void Remote_Control::SUB_SPEED_CONTROL(bool& print, bool& state, int button, int On, int Off, double& scale, double delta, double min, double max)
{
	if(!state && button==On) {
		state=true;
		print=true;
		double scale_temp=scale + delta;
		if(scale_temp < min) scale=min;
		else if(scale_temp > max) scale=max;
		else scale=scale_temp;
	}
	else if(state && button==Off) state=false;
}

void Remote_Control::checkConnection(const ros::TimerEvent& event) {
	if ((ros::Time::now() - last_received_time).toSec() > parameters.connection_timeout) { 
        isConnected = false; 
	}
	if ((ros::Time::now() - last_received_time).toSec() > parameters.reset_mode_timeout) {  
        isResetMode = true;  
	}
	printf("timeout: [%.1fs] --> [%.1fs] isConnected[%d] --> [%.1fs] isResetMode[%d] \n", (ros::Time::now() - last_received_time).toSec(), parameters.connection_timeout, isConnected, parameters.reset_mode_timeout, isResetMode);
	if(!isConnected){
		m_dLinear_spd = 0.0;
		m_dAngular_spd = 0.0;
		if(isResetMode){m_nMode = 0.0; isResetMode = false;}
		geometry_msgs::Twist output;
		output.linear.x = m_dLinear_spd;
		output.angular.z = m_dAngular_spd;
		output.linear.z = m_nMode;
		control_pub.publish(output);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




int main(int argc, char** argv)
{
	ros::init(argc, argv, "remote_control");
	ros::NodeHandle nh;
	ROS_INFO("START THE REMOTE CONTROL!!!");
	Remote_Control Remote_Control(nh);

	ros::spin();
	return 0;
}
