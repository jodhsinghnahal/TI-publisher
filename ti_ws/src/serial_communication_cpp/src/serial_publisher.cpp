#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <iostream>

serial::Serial ser;

void setupSerial() {
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port ");
        ros::shutdown();
    }

    if(ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        ROS_ERROR_STREAM("Serial Port not initialized");
        ros::shutdown();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_to_ros_node");
    ros::NodeHandle nh;

    ros::Publisher serial_pub = nh.advertise<std_msgs::String>("serial_data", 1000);

    setupSerial();

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        if(ser.available()) {
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            serial_pub.publish(result);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    ser.close();
}
