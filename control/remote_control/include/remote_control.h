#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>

#define _BLACK_ 	"\x1b[30m"
#define _RED_ 		"\x1b[31m"
#define _GREEN_ 	"\x1b[32m"
#define _YELLOW_ 	"\x1b[33m"
#define _BLUE_		"\x1b[34m"
#define _MAGENTA_	"\x1b[35m"
#define _CYAN_ 		"\x1b[36m"
#define _WHITE_		"\x1b[37m"
#define _RESET_		"\x1b[0m"

namespace STATE
{
    const int OFF = 0;
    const int ON = 1;

    const int AUTO = 0;
    const int MANUAL = 1;

    const int JOY = 6;
    const int CROSS = 7;

    const int UP = 1;
    const int DOWN = -1;
    const int LEFT = 1;
    const int RIGHT = -1;
}

namespace TYPE{
    const int PS4 = 1;
}
// joy->buttons : [Index Button] = [0 A] [1 B] [2 X] [3 Y] [4 LB] [5 RB] [6 BACK] [7 START] [8 MODE] [9 Button stick left] [10 Button stick right]
// joy->axes : [Index Axis] = [0 Axis LR1] [1 Axis UP1] [2 LT] [3 Axis LR2] [4 Axis UD2] [5 RT] [6 cross(+) key L/R] [7 cross(+) key U/D]
namespace XBOX_BTN
{
    const int A = 0;
    const int B = 1;
    const int X = 2;
    const int Y = 3;
    const int LB = 4;
    const int RB = 5;
    const int back = 6;
    const int start = 7;
    const int mode = 8;
    const int Button_stick_L = 9;
    const int Button_stick_R = 10;    
}

namespace XBOX_AXIS
{
    const double Axis_LR1 = 0;
    const double Axis_UD1 = 1;
    const double LT = 2;
    const double Axis_LR2 = 3;
    const double Axis_UD2 = 4;
    const double RT = 5;
    const double Cross_key_LR = 6;
    const double Cross_key_UD = 7;
}

// joy->buttons : [Index Button] = [0 A] [1 B] [2 X] [3 Y] [4 LB] [5 RB] [6 BACK] [7 START] [8 MODE] [9 Button stick left] [10 Button stick right]
// joy->axes : [Index Axis] = [0 Axis LR1] [1 Axis UP1] [2 LT] [3 Axis LR2] [4 Axis UD2] [5 RT] [6 cross(+) key L/R] [7 cross(+) key U/D]
namespace PS4_BTN
{
    const int A = 1;
    const int B = 2;
    const int X = 0;
    const int Y = 3;
    const int LB = 4;
    const int RB = 5;
    const int LT = 6;
    const int RT = 7;
    const int back = 8;
    const int start = 9;
    const int mode = 12;
    const int Button_stick_L = 10;
    const int Button_stick_R = 11;
}

namespace PS4_AXIS
{
    const double Axis_LR1 = 0;
    const double Axis_UD1 = 1;
    const double Axis_LR2 = 3;
    const double Axis_UD2 = 4;
    const double Cross_key_LR = 6;
    const double Cross_key_UD = 7;
}


// joy->buttons : [Index Button] = [0 A] [1 B] [2 X] [3 Y] [4 LB] [5 RB] [6 BACK] [7 START] [8 MODE] [9 Button stick left] [10 Button stick right]
// joy->axes : [Index Axis] = [0 Axis LR1] [1 Axis UP1] [2 LT] [3 Axis LR2] [4 Axis UD2] [5 RT] [6 cross(+) key L/R] [7 cross(+) key U/D]
namespace LOGITECH_BTN
{
    const int A = 1;
    const int B = 2;
    const int X = 0;
    const int Y = 3;
    const int LB = 4;
    const int RB = 5;
    const int LT = 6;
    const int RT = 7;
    const int back = 8;
    const int start = 9;
    const int mode = 9;
    const int Button_stick_L = 10;
    const int Button_stick_R = 11;
}

namespace LOGITECH_AXIS
{
    const double Axis_LR1 = 0;
    const double Axis_UD1 = 1;
    const double Axis_LR2 = 2;
    const double Axis_UD2 = 3;
    const double Cross_key_LR = 4;
    const double Cross_key_UD = 5;
}
class Remote_Control
{
public:
    Remote_Control(ros::NodeHandle& nh);
    void initParam(ros::NodeHandle& n);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    typedef struct Parameters{
		double min_Linear;
        double Max_Linear;
        double min_Angular;
        double Max_Angular;
        int control_ratio;
        double linear_step_control_size;
        double angular_step_control_size;
        double init_linear_vel;
        double init_angular_vel;
        double init_angular_step_scale;
        double init_linear_step_scale;
        int mode_type;
        int key_type;
        double connection_timeout;
        double reset_mode_timeout;
	}Parameters;

    Parameters parameters;

private:
    

    bool E_stop(void);
    bool MODE(void);
    void CONTROL_MODE(void);
    void SPEED_CONTROL(void);
    void CALC_SPEED(void);
    void SUB_SPEED_CONTROL(bool& print, bool& state, int button, int On, int Off, double& scale, double delta, double min, double max);
    void checkConnection(const ros::TimerEvent& event);

    int test_data;

    // BUTTON
    int m_nCross_control;
    int m_nJoy_control;
    int m_nEstop;
    int m_nNoEstop;
    int m_nLinearSpeedDown;
    int m_nLinearSpeedUp;
    int m_nAngularSpeedDown;
    int m_nAngularSpeedUp;
    int m_nAutoMode;
    int m_nAutoBack;
    int m_nManualMode;
    int m_nStickButton_L;
    int m_nStickButton_R;

    // AXIS
    double m_dLinear_stick;
    double m_dAngular_stick;
    double m_dLinear_stick2;
    double m_dAngular_stick2;
    double m_nLinear_cross;
    double m_nAngular_cross;

    // Control output
    double m_dLinear_spd;
    double m_dAngular_spd;
    double m_dLinear_spd2;
    double m_dAngular_spd2;

    // States
    bool m_binitStart_joy;
    bool m_bEstop;
    bool m_bLinearSpeedDown;
    bool m_bLinearSpeedUp;
    bool m_bAngularSpeedDown;
    bool m_bAngularSpeedUp;
    bool m_bCrossUp;
    bool m_bCrossDown;
    bool m_bCrossLeft;
    bool m_bCrossRight;
    int m_nMode;
    int m_nControlMode;
    int m_nLinearStep;
    int m_nAngularStep;

    // Parameters
    double m_dmin_Linear;
    double m_dMax_Linear;
    double m_dmin_Angular;
    double m_dMax_Angular;
    /* Joy */
    int m_nControl_ratio;
    double m_dLinear_scale;
    double m_dAngular_scale;
    double m_dLinear_control;
    double m_dAngular_control;
    /* Cross */
    double m_dLinear_step_scale;
    double m_dAngular_step_scale;
    double m_dLinear_step_control;
    double m_dAngular_step_control;

};
