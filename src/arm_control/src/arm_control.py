#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class WheelState:
    def __init__(self):
        self.pub1 = rospy.Publisher('arms_vels', Int32MultiArray, queue_size=10)
        
        self.base_rpwm = 0  # Right wheel forward PWM
        self.base_lpwm = 0  # Right wheel backward PWM
        self.elbow_rpwm = 0  
        self.elbow_lpwm = 0  
        self.bevel_rpwm = 0
        self.bevel_lpwm = 0
        self.gripper_rpwm = 0
        self.gripper_lpwm = 0 
        self.close = 0
        self.open = 0
        
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        # Check joystick input and set wheel velocities accordingly
        if data.buttons[3] == 1:  # No lateral movement (turning)
            self.elbow_rpwm = 150
            self.elbow_lpwm = 0
        elif data.buttons[0] == 1:  # No lateral movement (turning)
            self.elbow_rpwm = 0
            self.elbow_lpwm = 150
        else :
            self.elbow_rpwm = 0
            self.elbow_lpwm = 0        
        if data.buttons[5] ==1:
            self.base_rpwm = 150
            self.base_lpwm = 0
        elif data.buttons[4] == 1:
            self.base_rpwm = 0
            self.base_lpwm = 150
        else: 
            self.base_lpwm = 0
            self.base_rpwm = 0   
        if data.buttons[2]==1:
            self.gripper_rpwm = 50
            self.gripper_lpwm = 0 
        elif data.buttons[1]==1:
            self.gripper_rpwm = 0 
            self.gripper_lpwm = 50
        else:
            self.gripper_lpwm = 0
            self.gripper_rpwm = 0 
        if data.axes[2]<0:     
             self.open = 1
        else :
             self.open = 0
        if data.axes[5]<0:     
             self.close = 1
        else :
             self.close = 0    
        if data.axes[7]== 1:     
             self.bevel_rpwm = 150
             self.bevel_lpwm = 0
        elif data.axes[7] == -1:
             self.bevel_rpwm = 0
             self.bevel_lpwm = 150
        else :
             self.bevel_rpwm = 0
             self.bevel_lpwm = 0                               
         
        # Create and publish the wheel velocities array
        arms_vels_arr = [self.base_rpwm, self.base_lpwm, self.bevel_rpwm, self.bevel_lpwm, self.elbow_rpwm, self.elbow_lpwm, self.gripper_rpwm, self.gripper_lpwm, self.open, self.close]
        arms_data_to_send = Int32MultiArray()
        arms_data_to_send.data = arms_vels_arr
        self.pub1.publish(arms_data_to_send)

        # Debugging output
        rospy.loginfo(f"Published arm velocities: {arms_vels_arr}")

def main():
    rospy.init_node('joy_to_arm', anonymous=True)
    wheel_state = WheelState()
    rospy.spin()

if __name__ == '__main__':
    main()
