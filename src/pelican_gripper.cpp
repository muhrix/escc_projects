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

int fd;
int ctrl_switch;

//for (int i = 0; i < cnt; i++) {
//    write(fd, &tbyte[i], 1);
//}

void callback(const asctec_hlp_comm::mav_rcdataConstPtr& msg) {
    if (msg->channel.at(ctrl_switch) < 0.0) {

    }
    else if (msg->channel.at(ctrl_switch) > 0.0) {

    }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "gripper");
  ros::NodeHandle n("~");
  std::string port;
  int baud;
  n.param<std::string>("port", port, std::string("/dev/ttyACM0"));
  n.param<int>("baud", baud, int(57600));
  n.param<int>("gripper_switch", ctrl_switch, int(6));

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

  ros::Subscriber sub = n.subscribe<asctec_hlp_comm::mav_rcdata>("rc_data", 1, callback);

  ros::spin();
  return 0;
}
