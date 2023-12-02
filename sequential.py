#!/usr/bin/env python3

from typing import Any
import rospy
from std_msgs.msg import Float32MultiArray, Float64 ,Bool, Int32
from geometry_msgs.msg import Twist , Pose , PoseStamped
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL , CommandBool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import math


class node:
    def __init__(self):
        rospy.init_node('sequential_node',anonymous=True)
        #rospy.Subscriber('/detector_node',Float32MultiArray,self.callback)
        #rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,self.callback2)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64,callback=self.callback2)
        rospy.Subscriber('/mavros/global_position/global',NavSatFix, callback=self.callback)
        rospy.Subscriber('/mavros/global_position/local',Odometry,callback=self.callback3)
        
        self.publisher = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.publisher2 = rospy.Publisher('sequential',Int32,queue_size=10)
        self.pilot_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.pilot_tol = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
        self.arm_motors = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
        self.alt = 0.0
        self.gps = NavSatFix()
        self.localpos = Odometry()
        self.getpose = [0.0, 0.0, 0.0]
        self.state = Int32()
       

    def callback(self,data):
        self.gps = data
    def callback2(self,data):
        self.altitude = data.data
    def callback3(self,data):
        self.localpos = data

    

   

    def publish(self):
        msg = PoseStamped()
        msg.pose.position.x = self.getpose[0]
        msg.pose.position.y = self.getpose[1]
        msg.pose.position.z = self.getpose[2]
        self.publisher.publish(msg)

    def publish2(self):
        msg = Int32()
        msg = pilot_node.state
        self.publisher2.publish(msg)


if __name__ == '__main__':

    #------------ Inicio ----------------------
    state = 0
    pilot_node = node()
    rate = rospy.Rate(10)
    takeoff_altitude = 15

    x_localp = -13.3506
    y_localp = 201.944

    pilot_node.state.data = state
    

    while not rospy.is_shutdown():

        # ------- Estado actual ---------
        state0 = state == 0
        state1 = state == 1
        state2 = state == 2
        state3 = state == 3


        # --------- Armando motores (estado 1) --------------

        if state0 :
            pilot_node.pilot_mode(custom_mode = 'GUIDED')
            print('Armando motores ......')
            rospy.sleep(3.)
            pilot_node.arm_motors(True)
            print('\n \n ')
            print('-----------------')
            print(' Motores Armados ')
            print('-----------------')
            state = 1
        elif state1 :
            print ('Despegando')
            lat = pilot_node.gps.latitude
            lon = pilot_node.gps.longitude
            pilot_node.pilot_tol(0.0 , 0.0, lat, lon, takeoff_altitude)
            if pilot_node.altitude >= takeoff_altitude:
                print('Altura objetivo Alcanzada')
                state = 2
                rospy.sleep(2.)
        elif state2 :
            print('viajando..')
            error_x = x_localp - pilot_node.localpos.pose.pose.position.x
            error_y = y_localp - pilot_node.localpos.pose.pose.position.y

            error_x = math.pow(error_x,2)
            error_y = math.pow(error_y,2)
            error = math.sqrt(error_x + error_y)
            if error <= 1:
                print('punto alcanzado')
                rospy.sleep(2.)
                state = 3
            else:
                pilot_node.getpose[0] = x_localp
                pilot_node.getpose[1] = y_localp
                pilot_node.getpose[2] = takeoff_altitude

                pilot_node.publish()

            


            

        pilot_node.state.data = state
        pilot_node.publish2()
        rate.sleep()

       



