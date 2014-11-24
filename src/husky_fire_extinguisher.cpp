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

// Shift key turns on the pump
// Ctrl key turns off the pump
// Right key selects the motor ID 1
// Left key selects the motor ID 2
// Up key moves motor
// Down key moves motor

int fd;
int pan, tilt, pump;

void callback(const sensor_msgs::JoyConstPtr& msg) {
    unsigned char cmd;
    if (msg->buttons.at(pump) == 1) {
        // write command to serial port to activate pump
    }
    else if (msg->buttons.at(pump) == 0) {
        // write command to turn off water pump
    }
    if (msg->axes.at(pan) > 0.0) {
        // move right (send right arrow character)

    }
    else if (msg->axes.at(pan) < 0.0) {
        // move left (send left arrow character)
    }
    if (msg->axes.at(tilt) > 0.0) {
        // move down
    }
    else if (msg->axes.at(tilt) < 0.0) {
        // move up
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "fire_extinguisher");
    ros::NodeHandle n("~");
    std::string port;
    int baud;
    n.param<std::string>("port", port, std::string("/dev/ttyUSB0"));
    n.param<int>("baud", baud, int(57600));
    n.param<int>("pan_axis", pan, int(2));
    n.param<int>("tilt_axis", tilt, int(3));
    n.param<int>("pump_button", pump, int(3));

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

    ros::spin();

    close(fd);

    return 0;
}
