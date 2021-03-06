/*
 * pelican_gripper.cpp
 *
 *  Created on: 24 Nov 2014
 *      Author: Murilo F. M.
 *      Email:  muhrix@gmail.com
 *
 */

#include <ros/ros.h>
#include <asctec_hlp_comm/mav_rcdata.h>

#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h>  // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <pthread.h>

int fd;
int ctrl_switch;
unsigned char cmd;
unsigned char buf[10];
int bytes_read;
int counter = 0;

int arduino_map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// I must send 48 as the "wide-open position", whilst 54 is the maximum range
// for the gripper to be closed...
// 48 is equivalent to 0, whilst 54 is equivalent to 120

void callback(const asctec_hlp_comm::mav_rcdataConstPtr& msg) {
    if (counter > 5) {
        counter = 0;
        /*
        if (msg->channel.at(ctrl_switch) > 3100) {
            cmd--;
            if (cmd < 48) {
                cmd = 48;
            }
            ROS_INFO("Closing gripper: %d", int(cmd));
        }
        else if (msg->channel.at(ctrl_switch) < 800) {
            cmd++;
            ROS_INFO("Opening gripper: %d", int(cmd));
        }
        // write byte
        write(fd, &cmd, 1);
        // read byte
        //bytes_read = read(fd, buf, 10);
        //ROS_INFO("Sonar range: %d", int(buf[0]));
        */
        // the implementation below by-passes the incremental step control
        // and uses the know switch value to interpolate between 0 and 120
        cmd = (unsigned char)(arduino_map(msg->channel.at(ctrl_switch),
                                          0, 4096, 0, 120));
        write(fd, &cmd, 1);
    }
    else {
        counter++;
    }    
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "gripper");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    std::string port;
    int baud;
    n_priv.param<std::string>("port", port, std::string("/dev/ttyACM0"));
    n_priv.param<int>("baud", baud, int(115200));
    n_priv.param<int>("gripper_switch", ctrl_switch, int(6));

    cmd = 0x30; //48;

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

    ros::Subscriber sub = n.subscribe<asctec_hlp_comm::mav_rcdata>("rcdata", 1, callback);

    ros::spin();

    close(fd);

    return 0;
}
