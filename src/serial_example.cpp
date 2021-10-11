#include <ostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include <queue>

#include "fr_serialcomm_ros/serial_message.h"

#include "ros/publisher.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "ros/ros.h"
#include "serial/serial.h"

#define BAUDRATE 115200

std::string detectPort()
{
    std::string uno = "2341:0043";                       // vid/pid for an Arduino Uno
    std::string port;
    std::string ids;

    // Goes through through all the ports and checks for a device with a vid_pid of an Arduino Uno
    std::vector<serial::PortInfo> devices_found = serial::list_ports();

    for (serial::PortInfo &element : devices_found)
    {
        if (element.hardware_id != "n/a")
        {
            ids = element.hardware_id;
            if (ids.find(uno) != std::string::npos)
            {
                port = element.port;
                std::cout << "Accepted port as arduino UNO:" << ids;
                return port;
            }

            std::string ids = element.hardware_id;
            std::cout << "Detail port:" << ids;
        }
    }
    std::cout << "We could not find an arduino uno to connect to, if you have a different microcontroller, please modify the detectPort() function to your specifications" << ids;
    throw "No port found matching description!";
}



int main(int argc, char **argv)
{

    std::cout << "Started Node" << std::endl;
    std::string port = detectPort();
    std::cout << "Port for Arduino Uno is" << port << std::endl;
    ros::init(argc, argv, "Randle_Serial_Interface");
    ros::NodeHandle node;
    ros::Rate rate(1000);

    ros::Publisher system_publisher = node.advertise<fr_serialcomm_ros::serial_message>("randle_state", 10);

    //In this section we open up the port on the appropraite baud rate to listen to the autodetected microcontroller and establish a connection
    serial::Serial _serial(port, BAUDRATE, serial::Timeout::simpleTimeout(1000));

    if (_serial.isOpen())
    {
        _serial.flush();
        _serial.flushInput();
        _serial.flushOutput();
        std::cout << "Port is open and active" << std::endl;
    }
    else
    {
        std::cout << "Port was not opened correctly" << std::endl;
        return 0;
    }

    std::cout << "Starting Main Communication" << std::endl;
    std::string input;
        
    while (ros::ok())
    {
        std::cout << "Please enter a number between -10000-10000 to create a message, the terms 'send' to send the payload or 'exit' to quit the program" << port << std::endl;
        std::cin >> input;
        std::cout<<input;
        
        rate.sleep();
        ros::spinOnce();
    }
    _serial.close();
    return 0;
}