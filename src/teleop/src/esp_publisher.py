#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class WheelState:
    def __init__(self):
        self.pub1 = rospy.Publisher('wheel_vels', Int32MultiArray, queue_size=10)
        
        self.rwheel_rpwm = 0  # Right wheel forward PWM
        self.rwheel_lpwm = 0  # Right wheel backward PWM
        self.lwheel_rpwm = 0  # Left wheel forward PWM
        self.lwheel_lpwm = 0  # Left wheel backward PWM
        
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        # Check joystick input and set wheel velocities accordingly
        
        if data.axes[1]>0:  
                #move forward  
                self.rwheel_rpwm = floor(255 * data.axes[1])
                self.rwheel_lpwm = 0  
        elif data.axes[1]<0 :
                #move backward
                self.rwheel_rpwm = 0
                self.rwheel_lpwm = -floor(255 * data.axes[1])
        if data.axes[4]<0:  
                #move forward  
                self.lwheel_rpwm = -floor(255 * data.axes[4])
                self.lwheel_lpwm = 0  
        elif data.axes[4]>0 :
                #move backward
                self.lwheel_rpwm = 0
                self.lwheel_lpwm = floor(255 * data.axes[4])    
                

        # Create and publish the wheel velocities array
        wheel_vels_arr = [self.rwheel_rpwm, self.rwheel_lpwm, self.lwheel_rpwm, self.lwheel_lpwm]
        wheel_data_to_send = Int32MultiArray()
        wheel_data_to_send.data = wheel_vels_arr
        self.pub1.publish(wheel_data_to_send)

        # Debugging output
        rospy.loginfo(f"Published wheel velocities: {wheel_vels_arr}")

def main():
    rospy.init_node('joy_to_p2', anonymous=True)
    wheel_state = WheelState()
    rospy.spin()

if __name__ == '__main__':
    main()
