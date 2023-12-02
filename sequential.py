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

        # ------------------------ Subscribers -----------------------------
        #rospy.Subscriber('/detector_node',Float32MultiArray,self.callback)
        #rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,self.callback2)
        rospy.Subscriber('/mavros/global_position/rel_alt', Float64,callback=self.callback2)
        rospy.Subscriber('/mavros/global_position/global',NavSatFix, callback=self.callback)
        rospy.Subscriber('/mavros/global_position/local',Odometry,callback=self.callback3)
        rospy.Subscriber('/detector_node',Float32MultiArray,callback=self.callback4)
        
        #----------------------- Publishers --------------------------------
        self.publisher = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=10)
        self.publisher2 = rospy.Publisher('sequential',Int32,queue_size=10)
        self.pilot_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.pilot_tol = rospy.ServiceProxy('/mavros/cmd/takeoff',CommandTOL)
        self.arm_motors = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)

        #----------------------- Variables -----------------------------------
        self.alt = 0.0
        self.gps = NavSatFix()
        self.localpos = Odometry()
        self.getpose = [0.0, 0.0, 0.0]
        self.state = Int32()
        self.process_value = [0.0 , 0.0]
        self.compass = 0.0

    # --------- Callbacks functions -------------------
    def callback(self,data):
        self.gps = data
    def callback2(self,data):
        self.altitude = data.data
    def callback3(self,data):
        self.localpos = data
    def callback4(self,data):
        self.process_value = data.data

    

   

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

# ---- Funcion que advierte el inicio de un nuevo estado -----
def new_state_alert(actual_state):
    print("\n")
    print("\n")
    print("-----------------")
    if actual_state == 0:
        print("[INFO]: Initializing STATE 0")
    if actual_state == 1:
        print("[INFO]: Initializing STATE 1")
    if actual_state == 2:
        print("[INFO]: Initializing STATE 2")
    if actual_state == 3:
        print("[INFO]: Initializing STATE 3")
    print("-----------------")



if __name__ == '__main__':

    #------------ Inicio ----------------------
    state = 0

    pilot_node = node()
    rate = rospy.Rate(10)
    takeoff_altitude = 15

    x_localp = -13.3506
    y_localp = 201.944

    x_control = controller()
    y_control = controller()

    state3_flag = False #----> Bandera de inicio de estado 3

    #----- Control de programa -----
    flag_orientation = False

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

        # ------- Despegue (estado 2) -------
        elif state1 :
            print ('Despegando')
            lat = pilot_node.gps.latitude
            lon = pilot_node.gps.longitude
            pilot_node.pilot_tol(0.0 , 0.0, lat, lon, takeoff_altitude)
            if pilot_node.altitude >= takeoff_altitude:
                print('Altura objetivo Alcanzada')
                state = 2
                rospy.sleep(2.)

        # ---- Desplazamiento (estado 3) --------
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

        elif state3:
            if not state3_flag:
                state3_flag = True
                new_state_alert(3) #----> Funcion de inicio de estado

            


            

        pilot_node.state.data = state
        pilot_node.publish2()
        rate.sleep()

       



