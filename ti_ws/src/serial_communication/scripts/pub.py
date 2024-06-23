#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Int32MultiArray

def process_line(line):
    parts = line.decode('utf-8').strip().split()
    
    if len(parts) != 18:
        raise ValueError("Line does not contain the expected number of elements")
    
    values = [int(part) for part in parts]
    
    pot_values = values[:16]
    
    button_states_1 = values[16]
    button_states_2 = values[17]
    
    button_states_1_bin = format(button_states_1, '032b')[::-1]
    button_states_2_bin = format(button_states_2, '032b')[::-1]
    
    buttons_1 = [int(bit) for bit in button_states_1_bin]
    buttons_2 = [int(bit) for bit in button_states_2_bin]
    
    return pot_values, buttons_1, buttons_2

def main():
    rospy.init_node('serial_publisher', anonymous=True)
    
    pot_pub = rospy.Publisher('/pot_values', Int32MultiArray, queue_size=10)
    buttons_pub_1 = rospy.Publisher('/buttons_1', Int32MultiArray, queue_size=10)
    buttons_pub_2 = rospy.Publisher('/buttons_2', Int32MultiArray, queue_size=10)
    
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(1)
    ser.reset_input_buffer()
    print("SERIAL OK")

    rate = rospy.Rate(500)  # Increase rate to 50 Hz
    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline()
            
            try:
                pot_values, buttons_1, buttons_2 = process_line(line)
                
                pot_msg = Int32MultiArray(data=pot_values)
                buttons_msg_1 = Int32MultiArray(data=buttons_1)
                buttons_msg_2 = Int32MultiArray(data=buttons_2)
                
                pot_pub.publish(pot_msg)
                buttons_pub_1.publish(buttons_msg_1)
                buttons_pub_2.publish(buttons_msg_2)
                
                print("Potentiometer values:", pot_values)
                print("Button states from first int:", buttons_1)
                print("Button states from second int:", buttons_2)
                print()
            except ValueError as e:
                print(f"Error processing line: {e}")
        
        rate.sleep()

if __name__ == "__main__":
    main()

