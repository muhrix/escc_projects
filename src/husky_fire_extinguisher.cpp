/*
 * husky_fire_extinguisher.cpp
 *
 *  Created on: 24 Nov 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions

// Shift key turns on the pump -- serial.write(124)
// Ctrl key turns off the pump -- serial.write(123);
// Right key selects the motor ID 1 -- serial.write(121)
// Left key selects the motor ID 2  -- serial.write(122)
// Up key moves motor
// Down key moves motor
// maximum value is 120 and step is 2

// pan is motor ID 2
// tilt is motor ID 1

int fd;
int pan, tilt, pump;
int pan_left, pan_right;
int tilt_up, tilt_down;
bool pump_on = false;
int counter = 0;
unsigned char pan_cmd = 0x00;
unsigned char tilt_cmd = 0x00;

void callback(const sensor_msgs::JoyConstPtr& msg) {
    unsigned char cmd;
    if (msg->buttons.at(pump) == 1) {
        if (pump_on == false) {
            // write command to serial port to activate pump
            cmd = 0x7C; //124;
            write(fd, &cmd, 1);
            ROS_INFO("Pump command: %d", int(cmd));
            pump_on = true;
        }
    }
    else if (msg->buttons.at(pump) == 0) {
        if (pump_on == true) {
            // write command to turn off water pump
            cmd = 0x7B; //123;
            write(fd, &cmd, 1);
            ROS_INFO("Pump command: %d", int(cmd));
            pump_on = false;
        }
    }
    if (msg->buttons.at(pan_left) == 1 &&
            msg->buttons.at(pan_right) == 0) {
        // firstly, choose motor ID 2
        cmd = 0x7A; //122;
        write(fd, &cmd, 1);
        ROS_INFO("Motor ID command: %d", int(cmd));
        // then send actual angle
        pan_cmd += 2;
        if (pan_cmd > 120) {
            pan_cmd = 118;
        }
        write(fd, &pan_cmd, 1);
        ROS_INFO("Pan command: %d", int(pan_cmd));
    }
    if (msg->buttons.at(pan_right) == 1 &&
            msg->buttons.at(pan_left) == 0) {
        // firstly, choose motor ID 2
        cmd = 0x7A; //122;
        write(fd, &cmd, 1);
        ROS_INFO("Motor ID command: %d", int(cmd));
        // then send actual angle
        pan_cmd -= 2;
        if (pan_cmd < 0) {
            pan_cmd = 0;
        }
        write(fd, &pan_cmd, 1);
        ROS_INFO("Pan command: %d", int(pan_cmd));
    }
    if (msg->buttons.at(tilt_up) == 1 &&
            msg->buttons.at(tilt_down) == 0) {
        // firstly, choose motor ID 1
        cmd = 0x79; //121;
        write(fd, &cmd, 1);
        ROS_INFO("Motor ID command: %d", int(cmd));
        // then send actual angle
        tilt_cmd += 2;
        if (tilt_cmd > 120) {
            tilt_cmd = 118;
        }
        write(fd, &tilt_cmd, 1);
        ROS_INFO("Tilt command: %d", int(tilt_cmd));
    }
    if (msg->buttons.at(tilt_down) == 1 &&
            msg->buttons.at(tilt_up) == 0) {
        // firstly, choose motor ID 1
        cmd = 0x79; //121;
        write(fd, &cmd, 1);
        ROS_INFO("Motor ID command: %d", int(cmd));
        // then send actual angle
        tilt_cmd -= 2;
        if (tilt_cmd < 0) {
            tilt_cmd = 0;
        }
        write(fd, &tilt_cmd, 1);
        ROS_INFO("Tilt command: %d", int(tilt_cmd));
    }
    /*
    if (msg->axes.at(pan) != 0.0) {
        // firstly, choose motor ID 2
        cmd = 0x7A; //122;
        write(fd, &cmd, 1);
        // then send actual angle
        // left-most joystick  = 1.0
        // right-most joystick = -1.0
        // left-most value  = 0
        // right-most value = 120
        int val = 59 - int(msg->axes.at(pan) * 59);
        cmd = (unsigned char)(val);
        write(fd, &cmd, 1);
        ROS_INFO("Pan command: %d", val);
    }
    if (msg->axes.at(tilt) != 0.0) {
        // firstly, choose motor ID 1
        cmd = 0x79; //121;
        write(fd, &cmd, 1);
        // then send actual angle
        // up-most joystick   = 1.0
        // down-most joystick = -1.0
        // up-most value   = 120
        // down-most value = 0
        int val = 59 + int(msg->axes.at(tilt) * 59);
        cmd = (unsigned char)(val);
        write(fd, &cmd, 1);
        ROS_INFO("Tilt command: %d", val);
    }
    */
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fire_extinguisher");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    std::string port;
    int baud;
    n_priv.param<std::string>("port", port, std::string("/dev/ttyUSB2"));
    n_priv.param<int>("baud", baud, int(9600));
    n_priv.param<int>("pan_axis", pan, int(2));
    n_priv.param<int>("tilt_axis", tilt, int(3));
    n_priv.param<int>("pump_button", pump, int(3));
    n_priv.param<int>("pan_left", pan_left, int(4));
    n_priv.param<int>("pan_right", pan_right, int(6));
    n_priv.param<int>("tilt_up", tilt_up, int(5));
    n_priv.param<int>("tilt_down", tilt_down, int(7));

    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    struct termios port_settings; // structure to store the port settings in

    switch (baud) {
    case 9600:
        cfsetispeed(&port_settings, B9600);    // set baud rates
        port_settings.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
        break;
    case 57600:
        cfsetispeed(&port_settings, B57600);    // set baud rates
        port_settings.c_cflag = B57600 | CS8 | CREAD | CLOCAL;
        break;
    case 115200:
        cfsetispeed(&port_settings, B115200);    // set baud rates
        port_settings.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
        break;
    case 230400:
        cfsetispeed(&port_settings, B230400);    // set baud rates
        port_settings.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
        break;
    default:
        cfsetispeed(&port_settings, B115200);    // set baud rates
        port_settings.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
    }
    //cfsetispeed(&port_settings, B230400);    // set baud rates
    //port_settings.c_cflag = B230400 | CS8 | CREAD | CLOCAL;
    port_settings.c_iflag = IGNPAR;
    port_settings.c_oflag = 0;
    port_settings.c_lflag = 0;
    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

    ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1, callback);

    ROS_INFO("Fire extinguisher node started");
    ros::spin();

    close(fd);

    return 0;
}
