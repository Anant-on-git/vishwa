#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy
from math import floor

class WheelState:
    def __init__(self):
        self.pub1 = rospy.Publisher('science_vels', Int32MultiArray, queue_size=10)
        
        self.nema_17_up = 0
        self.nema_17_down = 0  
        self.worm_m1_open = 0  
        self.worm_m1_close = 0  
        self.worm_m2_open = 0  
        self.worm_m2_close = 0
        self.servo_left = 0
        self.servo_right = 0
        self.pump1 = 0
        self.pump2 = 0
        self.pump3 = 0
        self.pump4 = 0
        
        rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        # Check joystick input and set wheel velocities accordingly

        if data.buttons[2]==1:
           self.pump1 = 255
        else:
           self.pump1 = 0
        if data.buttons[0]==1:
           self.pump2 = 140
        else:
           self.pump2 = 0
        if data.buttons[1]==1:
           self.pump3 = 140
        else:
           self.pump3 = 0
        if data.buttons[3]==1:
           self.pump4 = 140
        else:
           self.pump4 = 0

       
        if data.buttons[4]==1:
             self.worm_m1_close = 200
        else:
             self.worm_m1_close = 0    
        if data.buttons[6]==1:
              self.worm_m1_open = 200
        else:
            self.worm_m1_open = 0      
        if data.buttons[5]==1:
             self.worm_m2_close = 200
        else:
             self.worm_m2_close = 0    
        if data.buttons[7]==1:
              self.worm_m2_open = 200
        else:
            self.worm_m2_open = 0          
        

        if data.buttons[16]==1 : 
            self.servo_left = 1
        else :
            self.servo_left = 0
            
        if data.buttons[15]==1 :
            self.servo_right = 1   
        else:
            self.servo_right = 0  


        self.nema_17_up = data.buttons[13]
        self.nema_17_down = data.buttons[14]
         
        # Create and publish the wheel velocities array
        science_vels_arr = [self.nema_17_up, self.nema_17_down, self.worm_m1_open, self.worm_m1_close, self.worm_m2_open, self.worm_m2_close, self.servo_left, self.pump1, self.pump2, self.pump3, self.pump4, self.servo_right]
        science_data_to_send = Int32MultiArray()
        science_data_to_send.data = science_vels_arr
        self.pub1.publish(science_data_to_send)

        # Debugging output
        rospy.loginfo(f"Published velocities: {science_vels_arr}")

def main():
    rospy.init_node('joy_to_science', anonymous=True)
    wheel_state = WheelState()
    rospy.spin()

if __name__ == '__main__':
    main()
