/***
 * This example expects the serial port has a loopback on it.
 *
 * Alternatively, you could use an Arduino:
 *
 * <pre>
 *  void setup() {
 *    Serial.begin(<insert your baudrate here>);
 *  }
 *
 *  void loop() {
 *    if (Serial.available()) {
 *      Serial.write(Serial.read());
 *    }
 *  }
 * </pre>
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/input.h>

serial::Serial ser;

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example");
    ros::NodeHandle nh;

    ros::Publisher read_pub = nh.advertise<std_msgs::String>("mag/hmr", 1);

    try
    {
        ser.setPort("/dev/HMR");
        ser.setBaudrate(19200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate rate(9);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()>=13){
            std_msgs::String output;
            
            output.data = ser.readline(ser.available());
            read_pub.publish(output);

        }
        rate.sleep();

    }
}
