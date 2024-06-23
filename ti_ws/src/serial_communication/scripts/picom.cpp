#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <array>

std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
            result += buffer.data();
        }
    }
    return result;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "picocom_to_ros_node");
    ros::NodeHandle nh;

    ros::Publisher serial_pub = nh.advertise<std_msgs::String>("serial_data", 1000);

    ros::Rate loop_rate(10);

    const char* cmd = "sudo picocom -b 9600 /dev/ttyUSB0";

    while (ros::ok()) {
        std::string serial_output = exec(cmd);
        if (!serial_output.empty()) {
            std_msgs::String msg;
            msg.data = serial_output;
            ROS_INFO_STREAM("Read: " << msg.data);
            serial_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

