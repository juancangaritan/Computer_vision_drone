#!/usr/bin/env python3

from typing import Any
import rospy
from std_msgs.msg import Float32MultiArray, Float64, Int32
from geometry_msgs.msg import Twist , Pose
from mavros_msgs.srv import SetMode




class node:
    def __init__(self):
        rospy.init_node('controller_node',anonymous=False)
        rospy.Subscriber('/detector_node',Float32MultiArray,self.callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,self.callback2)
        rospy.Subscriber('/sequential', Int32,callback=self.callback3)
        
        
        self.publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=10)
        self.process_value = [0.0 , 0.0]
        self.compass = 0.0
        self.u_action = [0.0 , 0.0 , 0.0, 0.0]
        self.pilot_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.state = Int32()
        

    def callback(self,data):
        self.process_value = data.data

    def callback2(self,data):
        self.compass = data.data

    def callback3(self, data):
        self.state = data.data

    def callback4(self,data) :
        self.coordinates = data

    def publish(self):
        msg = Twist()
        msg.linear.x = self.u_action[0]
        msg.linear.y = self.u_action[1]
        msg.angular.z = self.u_action[2]
        msg.linear.z = self.u_action[3]
        
        self.publisher.publish(msg)

    def publish2(self):
        msg = Pose()
        

class controller:
    def __init__(self):
        self.controller_pv = 0.0
        self.output = 0.0
        self.constant = 0.0
        self.error = 0.0
        self.flag = False
        self.action_flag = False
        

    def process(self):
        self.error = 320 - self.controller_pv
        negative_error = self.error <= -10.0 
        positive_error = self.error >= 10.0 
        if not self.flag:
            if negative_error: self.output = 0.5 ; self.action_flag = True
            elif positive_error: self.output = -0.5 ; self.action_flag = True
            else: self.output = 0.0 ; self.flag = True 

    
        
def ajust_orientation(orientation):
    
    high_tolerance = orientation >= 355.0 and orientation<= 360.0
    low_tolerance = orientation <= 5.0 and orientation >= 0.0
    spin_pos = orientation < 180.0 
    spin_neg = orientation >= 180.0
    if high_tolerance or low_tolerance : 
        output = 0.0 
        flag_orientation = True
    elif spin_pos: output = 0.25 ; flag_orientation = False
    elif spin_neg: output = -0.25 ; flag_orientation = False
    return flag_orientation, output



    

if __name__ == '__main__':
    control_nodo = node()
    x_control = controller()
    y_control = controller()
    rate = rospy.Rate(10)
    flag_orientation = False
    init_flag = True
    init_flag2 = True
    takeoff_flag = True
    trip_flag = True
    arm_flag = True

    try:

        while not rospy.is_shutdown():
            execute_controller = control_nodo.state == 3
           
            if flag_orientation and execute_controller:
                if init_flag2:
                    rospy.sleep(3)
                    init_flag2 = False
                x_control.controller_pv = control_nodo.process_value[0]
                x_control.process()
                #print('error en x = {}'.format(x_control.error))
                print('Alineandose con pad')
                control_nodo.u_action[0] = x_control.output

                y_control.controller_pv = control_nodo.process_value[1]
                y_control.process()
                control_nodo.u_action[1] = -y_control.output

                if x_control.flag and y_control.flag and (x_control.action_flag or y_control.action_flag):
                    rospy.sleep(5.)
                    x_control.flag = False
                    y_control.flag = False
                    x_control.action_flag = False
                    y_control.action_flag = False

                if x_control.error<=10.0 and x_control.error>=-10 and y_control.error<=10.0 and y_control.error>=-10 : control_nodo.pilot_mode(custom_mode='LAND')

            elif not flag_orientation and execute_controller:
                print('Ajustando orientacion')
                if init_flag :
                    rospy.sleep(5.)
                    init_flag = False
                flag_orientation, spin_cmd = ajust_orientation(control_nodo.compass)
                control_nodo.u_action[2] = spin_cmd

            #--- publicar ------
            if execute_controller:
                control_nodo.publish()


            rate.sleep()

    except rospy.ROSException as e:

        print(e)



