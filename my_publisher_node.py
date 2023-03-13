#!/usr/bin/env python3
import os
import numpy as np
import rospy
import smbus2
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from smbus2 import SMBus
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from sensor_msgs.msg import Range
import time
speed = WheelsCmdStamped()


class MyPublisherNode(DTROS):
    def __init__(self, node_name):
        # initialize parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)

        
        #----------------------------PUBLISHERS AND SUBSCRIBERS--------------------------------------------------------------
        
        
        self.pub = rospy.Publisher('/shelby/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=10)
        self.tof = rospy.Subscriber('/shelby/front_center_tof_driver_node/range', Range, self.callback)
        self.rwheel = rospy.Subscriber('/shelby/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.rightwheel)
        self.lwheel = rospy.Subscriber('/shelby/left_wheel_encoder_node/tick', WheelEncoderStamped, self.leftwheel)
        self.seqLeft = rospy.Subscriber('/shelby/left_wheel_encoder_node/tick', WheelEncoderStamped, self.time_leftwheel)
        self.seqRight = rospy.Subscriber('/shelby/right_wheel_encoder_node/tick', WheelEncoderStamped ,self.time_rightwheel)

        
        #--------------------------------------------VARIABLES----------------------------------------------------------------
        
        
        self.bus = SMBus(12)
        self.range = 1
        self.right = 0
        self.left = 0
        self.ticks_left = 0
        self.prev_tick_left = 0
        self.ticks_right = 0
        self.prev_tick_right = 0
        self.rotation_wheel_left = 0
        self.rotation_wheel_right = 0
        self.delta_ticks_left = 0
        self.delta_ticks_right = 0
        self.n_tot = 135
        self.last_error = 0
        self.delta_time = 1/20
        self.prev_integral = 0
        self.previous_left = 0
        self.previous_right = 0
        self.wtraveltmp = 0
        self.distance_cm = 0
        self.wtravel = 0
        
        
        #---------------------------------------------------------------------------------------------------------------------
    
    
    def callback(self, data):
        self.range = data.range

    def rightwheel(self, data):
        self.right = data.data

    def leftwheel(self, data):
        self.left = data.data
        
        
        #-------------------------------------OBSTACLE AVOIDANCE--------------------------------------------------------------
        
        
    def obstacle():
        while self.distance_cm < 35: #if obstacle detected within 35cm range, robot turns right
            speed.vel_left = 0.33
            speed.vel_right = 0.05
            self.pub.publish(speed)
            self.distance_cm = round(self.range*100, 1)
        time.sleep(0.2)
        while self.wtraveltmp < 30: #robot travels straight for 30cm
            speed.vel_left = 0.3
            speed.vel_right = 0.3
            self.pub.publish(speed)
            self.wtraveltmp = self.wtraveltmp + self.wtravel
        time.sleep(0.2)
        speed.vel_left = 0.05 #robot turns left
        speed.vel_right = 0.4
        self.pub.publish(speed)
        time.sleep(1.2)
        speed.vel_left = 0.3  #robot turns right
        speed.vel_right = 0.05
        time.sleep(0.5)

    def on_shutdown(self):
        speed.vel_left = 0
        speed.vel_right = 0
        self.pub.publish(speed)
        time.sleep(0.2)
        self.bus.close()
        rospy.on_shutdown()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            data = self.bus.read_byte_data(62,17)
            read = bin(data)[2:].zfill(8) # Reading binary data from a line counter, so the output value is always 8 digits
            R = 0.0318 #wheel radius
            kp = rospy.get_param("/p")
            ki = rospy.get_param("/i")
            kd = rospy.get_param("/d")


            #-----------------------------------------ODOMETRY EQUATIONS------------------------------------------------------------
            
            
            #Odometry equations for robot motion calculations
            self.ticks_right = self.right
            self.ticks_left = self.left
            self.delta_ticks_left = self.ticks_left-self.prev_tick_left
            self.delta_ticks_right = self.ticks_right-self.prev_tick_right
            self.rotation_wheel_left = (2*np.pi/self.n_tot)*self.delta_ticks_left #left wheel rotation angle (rad)
            self.rotation_wheel_right = (2*np.pi/self.n_tot)*self.delta_ticks_right #right wheel rotation angle (rad)
            d_left = R * self.rotation_wheel_left #distance travelled by left wheel (cm)
            d_right = R * self.rotation_wheel_right #distance travelled by right wheel (cm)
            self.prev_tick_left = self.ticks_left
            self.prev_tick_right = self.ticks_right
            self.wdistance = round(((d_left + d_right)*100)/2, 1) #distance travelled by robot (cm)
            self.distance_cm = round(self.range*100, 1) #distance detected by TOF sensor (cm)

            
            #-------------------------------------OBSTACLE AVIODANCE CALLOUT------------------------------------------------------------
            

            if self.distance_cm <= 35:
                obstacle()

                
            #------------------------------------------PID-CONTROLLER--------------------------------------------------------------------
            
                
            readings = [] #list of line sensor readings within 1-8 range
            for indx, nr in enumerate(data):
                if nr == "1":
                    readings.append(indx + 1)

            error = 4.5 - np.average(readings) #average of the line sensor readings subtracted from the center
            integral = self.prev_integral + error*self.delta_time
            integral = max(min(integral,2), -2) #limiting integral value
            derivative = (error - self.last_error)/self.delta_time
            correction = kp * error + ki * integral + kd * derivative #final equation for the PID controller 
            speed.vel_left = 0.5 - correction
            speed.vel_right = 0.5 + correction

            if len(readings) == 0: #if robot goes off the line, it continues moving based on the previous speed values
                speed.vel_left = self.previous_left
                speed.vel_right = self.previous_right
            self.previous_left = speed.vel_left
            self.previous_right = speed.vel_right

            speed.vel_left = max(0.0, min(speed.vel_left, 0.7)) #speedlimit
            speed.vel_right = max(0.0, min(speed.vel_right, 0.7)) #speedlimit
            self.pub.publish(speed)
            self.last_error = error
            rate.sleep()

if __name__ == '__main__':
    node = MyPublisherNode(node_name='my_publisher_node')
    rospy.on_shutdown(node.on_shutdown)
    node.run()
    rospy.spin()
