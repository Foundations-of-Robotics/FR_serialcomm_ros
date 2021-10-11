#include <ostream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <sstream>
#include <queue>
#include <list>
#include <unistd.h>

#include "fr_serialcomm_ros/serial_message.h"

#include "ros/publisher.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include "ros/ros.h"
#include "serial/serial.h"

#define BAUDRATE 115200
#define UPPER_BYTE(b) (b >> 8) //defines byte structure
#define LOWER_BYTE(b) (b & 0xff)
#define INT_JOIN_BYTE(u, l) (u << 8) | l

//A function to automatically to find the arduino device on USB
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
    //throw "No port found matching description!";
}


void ReadAndPublishData(ros::Publisher pub,  serial::Serial &_port){
    //Next we wait for the return message before publishing  
            uint8_t check_buffer[2];
            //while there is no serial data available, wait
            int counter=0;
            while(_port.available() == 0){
                std::cout << "..." << std::endl;
                usleep(100000);
                counter++;
                if(counter>100){
                    return;
                }
            }
            //read two bytes from array
            _port.read(check_buffer, 2);
            //Setup the header and payload variables from the read data
            uint8_t Header = (uint8_t)check_buffer[0];
            std::cout << "Header is: " << (int)Header << std::endl;
            uint8_t payload = (uint8_t)check_buffer[1];
            std::cout << "Payload is" << (int)payload << std::endl;
            // flushes if header value isn't correct
            //validate the header
            if (check_buffer[0] != 199)
            {
                std::cout << "Failed to read header, First Value is: " << (uint8_t)check_buffer[0] << std::endl;
                _port.flushInput();
                _port.flush();
                _port.flushOutput();
                ROS_ERROR_STREAM("Read front: check:" << check_buffer);
            }
            else
            {
                //read the payload and footer
                uint8_t message_buffer[payload+1];
                _port.read(message_buffer, payload+1);
                uint8_t footer = (uint8_t)message_buffer[payload];
                //print the recieved data from  the arduino
                for(int i = 0; i<sizeof(message_buffer);i++){
                    std::cout << "variable "<<(int)i<< "--"<< (int)message_buffer[i] << std::endl;
                }
                //validate the footter
                if (footer != 101){
                    std::cout << "Failed to read footer" << (int)footer << std::endl;
                    _port.flushInput();
                    _port.flush();
                    _port.flushOutput();
                    ROS_ERROR_STREAM("Read front: check:" << check_buffer);
                }else{
                    //Take the messages from the arduino and publish them to the rostopic /output_from_uno
                    fr_serialcomm_ros::serial_message data_recieved;
                    for (int i = 0; i < payload / 4; i++)
                    {
                        data_recieved.motorID = message_buffer[i * 4];
                        data_recieved.commandID = message_buffer[i * 4 + 1];
                        data_recieved.data = INT_JOIN_BYTE(message_buffer[i * 4 + 2], message_buffer[i * 4 + 3]);
                        pub.publish(data_recieved);
                    }
                }
            }  
}


int main(int argc, char **argv)
{
    /*This section of code detects the arduino automatically, makes a serial connection to the device and starts the ROS node*/
    std::cout << "Started Node" << std::endl;
    std::string port = detectPort();
    std::cout << "Port for Arduino Uno is" << port << std::endl;
    ros::init(argc, argv, "Randle_Serial_Interface");
    ros::NodeHandle node;
    ros::Rate rate(1000);


    /*Initializes the publisher for this node*/
    ros::Publisher system_publisher = node.advertise<fr_serialcomm_ros::serial_message>("output_from_uno", 10);

    //In this section we open up the port on the appropraite baud rate to listen to the autodetected microcontroller and establish a connection
    serial::Serial _serial(port, BAUDRATE, serial::Timeout::simpleTimeout(1000));

    if (_serial.isOpen())
    {
        //Clears the serial buffer on initialization
        _serial.flush();
        _serial.flushInput();
        _serial.flushOutput();
        //Debug Message
        std::cout << "Port is open and active" << std::endl;
    }
    else
    {
        //End program if the serial port was not opened correctly
        std::cout << "Port was not opened correctly" << std::endl;
        return 0;
    }

    
    std::cout << "Starting Main Communication" << std::endl;
    
    //the variable that holds the user input
    std::string input;
    
    //variable that limits the amount of messages we can send
    int sendingCounter = 0;

    //The array of bytes that we are going to send across to the arduino
    std::list<uint8_t> payloadArray;
    
    //A variable that holds the user input as an integer
    int data_as_Int;

    //A continous loop to operate
    while (ros::ok())
    {
        //user input commands
        std::cout  << "Please enter a number between -10000-10000 to create a message, the term 'send' to send the payload or 'quit' to exit the program: ";
        std::cin >> input;
        std::cout<<input  << std::endl;
        
        //saves input as an integer if applicable
        try { 
           data_as_Int = std::stoi( input );
         } catch (...) { 
             data_as_Int = 20000;
          }
        
        //if you enter send and have created messages, then this command will send data to the arduino
        if(input=="send" && payloadArray.size()>0){
            //sends data to arduino
            sendingCounter=0;
            uint8_t payloadSize = payloadArray.size()+1;
            payloadArray.push_front(payloadSize);
            payloadArray.push_front(199);
            payloadArray.push_back(101);
            uint8_t commandArray[payloadArray.size()];
            std::copy(payloadArray.begin(), payloadArray.end(), commandArray);
            _serial.write(commandArray, payloadArray.size()); 
            payloadArray.clear();
            //call the function to read data from the arduino and publish it from the system_publisher variable
            ReadAndPublishData(system_publisher,_serial);

        }else if(input=="quit"){
            //quits the node
            return 0;
        }else if((input=="0" || (data_as_Int<=10000 &&  data_as_Int>=-10000 && data_as_Int!=0))){
            //if the input is number, create a message
            if(sendingCounter>6){
                std::cout << "A maximum of 6 messages has been reached, please send the payload and try  again" << std::endl;
            }else{
            sendingCounter++;
            //assign byte arrays to program
            payloadArray.push_back(sendingCounter); //append a motor ID value
            payloadArray.push_back(sendingCounter+100); //append a second value incremented by 100
            payloadArray.push_back(UPPER_BYTE(data_as_Int));
            payloadArray.push_back(LOWER_BYTE(data_as_Int));
            std::cout << "Message Successfully appended" << std::endl;
            }

        }else{
            //invalid command
            std::cout << "Invalid command or data specified, or payload to send is empty" << std::endl;
        }
        //sleep and repeat loop
        rate.sleep();
        ros::spinOnce();
    }
    _serial.close();
    return 0;
}
