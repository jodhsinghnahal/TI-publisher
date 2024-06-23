#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import String

def serial_to_ros():
    # Initialize the ROS node
    rospy.init_node('serial_to_ros_node', anonymous=True)
    
    # Create a publisher object
    pub = rospy.Publisher('serial_data', String, queue_size=10)
    
    # Set the serial port and baud rate
    serial_port = '/dev/ttyUSB0'
    baud_rate = 9600
    
    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    
    rospy.loginfo(f"Opened serial port {serial_port} with baud rate {baud_rate}")

    # Set the loop rate
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        rospy.loginfo("hhi")
        if True:
            rospy.loginfo("kk")
            try:
                rospy.loginfo("ll")
                # Read a line from the serial port
                line = ser.readline()
                rospy.loginfo(f"Read from serial: {line}")
                
                # Publish the line to the ROS topic
                pub.publish(line)
                
            except serial.SerialException as e:
                rospy.logerr(f"Serial exception: {e}")
            except Exception as e:
                rospy.logerr(f"Unexpected error: {e}")

        # Sleep to maintain the loop rate
        rate.sleep()

    # Close the serial port on shutdown
    ser.close()
    rospy.loginfo("Closed serial port")

if __name__ == '__main__':
    try:
        serial_to_ros()
    except rospy.ROSInterruptException:
        pass

