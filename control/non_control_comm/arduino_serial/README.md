#### The first step

Install serial lib:
	- sudo apt-get install ros-melodic-serial
	- sudo apt-get install ros-melodic-rosserial
	- sudo chmod 666 /dev/ttyACM0 or permision for /dev/ttyACM0.
	- for keypad: dev/ttyACM0
	- for ultrasonic: dev/ttyUSB0

#### This is driver package

Run keypad: `roslaunch arduino_v1 arduino_serial.launch`

Run ultrasonic: `roslaunch arduino_v1 arduino_serial_sonara22.launch`

Run server: `roslaunch arduino_v1 arduino_serial_server.launch`

### DOCUMENTATION
##########################################
1. stpc
Subscribe topics:

-   `/robot_state` (std_msgs::String) from ..., user password data.

Publish topics:

-	send user password data when password was changed.

-   `/arduno_feedback` (std_msgs::Int16MultiArray) send system button, go button, door status data.

2. testing

- `rostopic pub /robot_state std_msgs/String "'5768,1'"`

- `rostopic pub /robot_id std_msgs/String "ID01"`

3. arduino info:

-	arduno_feedback data:
		data[0]: system button, default = 0;
		data[1]: go button, default = 0;
		data[2]: door status, lock = 1, unlock = 0;

-	robot state:
	switch (robot_state) 
    {
      case 0:
        mylcd.Print_String(" NOT READY  ", 175, 140);
        break;
      case 1:
        mylcd.Print_String("ROBOT READY ", 175, 140);
        break;
      case 2:
        mylcd.Print_String("  PASSWORD  ", 175, 140);
        break;
      case 8:
        mylcd.Print_String(" CONNECTED  ", 175, 140);
        break;
      case 9:
        mylcd.Print_String("DISCONNECTED", 175, 140);
        break;
      case 10:
        mylcd.Print_String("LOADING ARD ", 175, 140);
        break;  
      default:
        mylcd.Print_String(" UNKNOWN    ", 175, 140);
        break;
    }

-	stop_case:
	case 0:
		mylcd.Print_String("OK ", 100, 240); //NORMAL
		break;
	case 1:
		mylcd.Print_String("EST", 100, 240); //ESTOP_BTN
		break;
	case 2:
		mylcd.Print_String("GOB", 100, 240); //GO_BTN
		break;
	case 3:
		mylcd.Print_String("LOC", 100, 240); //LOC
		break;
	case 4:
		mylcd.Print_String("SYS", 100, 240); //SYSTEM
		break;
	case 5:
		mylcd.Print_String("ULT", 100, 240); //ULTRASONIC
		break;
	case 6:
		mylcd.Print_String("LTE", 100, 240); //LTE
		break;
	case 7:
		mylcd.Print_String("PER", 100, 240); //PER
		break;
	case 8:
		mylcd.Print_String("REM", 100, 240); //REMOTE
		break;  
	default:
		mylcd.Print_String(".. ", 100, 240); //UNKNOWN
		break;

-	sensor_state:
	switch (sensor_state) 
	{
	  case 0:
	    mylcd.Print_String("OK ", 100, 190); //NORMAL
	    break;
	  case 1:
	    mylcd.Print_String("XAV", 100, 190); //XAVIER
	    break;
	  case 2:
	    mylcd.Print_String("LID", 100, 190); //LIDAR
	    break;
	  case 3:
	    mylcd.Print_String("CAM", 100, 190); //CAM
	    break;
	  case 4:
	    mylcd.Print_String("SEG", 100, 190); //SEGWAY
	    break;
	  case 5:
	    mylcd.Print_String("LOC", 100, 190); //LOC
	    break;  
	  default:
	    mylcd.Print_String(".. ", 100, 190); //UNKNOWN
	    break;
	}
##########################################

1. stpc
Publish topics:

-   `/arduino_feedback_sonar` <std_msgs::Int16MultiArray> :
			data[0] = front_left_distance;
	    	data[1] = front_right_distance;
	    	data[2] = rear_left_distance;
	    	data[3] = rear_right_distance;
