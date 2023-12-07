#!/usr/bin/env python3

from typing import Any
import rospy
from std_msgs.msg import Float32MultiArray, Float64 ,Bool, Int32
from geometry_msgs.msg import TwistStamped , Pose , PoseStamped
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
        rospy.Subscriber('/mavros/global_position/compass_hdg',Float64,self.callback5)
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
        self.publisher3 = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)
        #----------------------- Variables -----------------------------------
        self.altitude = 0.0
        self.gps = NavSatFix()
        self.localpos = Odometry()
        self.getpose = [0.0, 0.0, 0.0]
        self.state = Int32()
        self.process_value = [0.0 , 0.0]
        self.compass = 0.0
        self.u_action = [0.0, 0.0, 0.0, 0.0] #---> accion de control

    # --------- Callbacks functions -------------------
    def callback(self,data): # ---> Datos GPS
        self.gps = data
    def callback2(self,data):# ---> Datos Altura
        self.altitude = data.data
    def callback3(self,data):# ---> Datos Odometro (IMU)
        self.localpos = data
    def callback4(self,data):#---> Posicion red neuronal (DNN)
        self.process_value = data.data
    def callback5(self,data):#---> Datos compas
        self.compass = data.data

    

   
# ----- Funciones publicar ------
    def publish(self): #----> 
        msg = PoseStamped()
        msg.pose.position.x = self.getpose[0]
        msg.pose.position.y = self.getpose[1]
        msg.pose.position.z = self.getpose[2]
        self.publisher.publish(msg)

    def publish2(self):
        msg = Int32()
        msg = pilot_node.state
        self.publisher2.publish(msg)

    def publish3(self):  # ---> Publicando velocidad
        msg = TwistStamped()
        msg.twist.linear.x = self.u_action[0]
        msg.twist.linear.y = self.u_action[1]
        msg.twist.linear.z = self.u_action[3]
        msg.twist.angular.z = self.u_action[2]
        self.publisher3.publish(msg)

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
        print("[INFO]: Current STATE 0")
    if actual_state == 1:
        print("[INFO]: Current STATE 1")
    if actual_state == 2:
        print("[INFO]: Current STATE 2")
    if actual_state == 3:
        print("[INFO]: Current STATE 3")
    print("-----------------")



if __name__ == '__main__':

    #------------ Inicio ----------------------
    state = 0
    flag_ini = True

    pilot_node = node()
    rate = rospy.Rate(10)
    takeoff_altitude = 15.0
    pilot_node.altitude = 0.0

    x_localp = -13.3506
    y_localp = 201.944

    x_control = controller()
    y_control = controller()

    state3_flag = False #----> Bandera de inicio de estado 3

    #---- Banderas de viaje -------
    flag_acel = True
    flag_const = False
    flag_desa = False

    #--- variable de viaje (speed) ----
    x_speed = 0.0

    #----- Control de programa -----
    flag_orientation = False

    pilot_node.state.data = state

    #------ Banderas de controlador -------
    flag_alinea1=True
    flag_decenso = False
    flag_alinea2 = False
    flag_Land = False
    

    while not rospy.is_shutdown():

        # ------- Estado actual ---------
        state0 = state == 0
        state1 = state == 1
        state2 = state == 2
        state3 = state == 3


        # --------- Armando motores (estado 1) --------------

        if state0 :
            if flag_ini:
                print('[INFO]: Initializing system')
                rospy.sleep(4.)
            new_state_alert(state)
            pilot_node.pilot_mode(custom_mode = 'GUIDED')
            if pilot_node.altitude < 0.5 :
                print('Armando motores ......')
                rospy.sleep(3.)
                pilot_node.arm_motors(True)
                print('\n \n ')
                print('-----------------')
                print(' Motores Armados ')
                print('-----------------')
                state = 1
            else:
                print('[ERROR]: Inconsistent altitude: {} m'.format(pilot_node.altitude))
        # ------- Despegue (estado 2) -------
        elif state1 :
            new_state_alert(state)
            print ('Despegando')
            print(pilot_node.altitude)
            lat = pilot_node.gps.latitude
            lon = pilot_node.gps.longitude
            pilot_node.pilot_tol(0.0 , 0.0, lat, lon, takeoff_altitude)
            if pilot_node.altitude >= takeoff_altitude:
                print('Altura objetivo Alcanzada')
                
                rospy.sleep(2.)
                state = 2

        # ---- Desplazamiento (estado 3) --------
        elif state2 :
            new_state_alert(state)
            #print('viajando..')
            if flag_acel:
                print('[STATUS]: Speeding up : {} m/s'.format(pilot_node.u_action[0]))
                if x_speed <= 15.0:
                    x_speed = x_speed + 0.1
                else: # -----> Fin de rampa 
                    flag_acel = False
                    flag_const = True 
            elif flag_const:
                x_speed = 5.0
                print('[STATUS]: Constant Speed : {} m/s'.format(x_speed))
                #rospy.sleep(5.)
                flag_const = False
                flag_desa = True
            elif flag_desa:
                print('[STATUS]: Slowing up : {} m/s'.format(x_speed))
                if x_speed > 0.0:
                    x_speed = x_speed - 0.25
                else: # -----> Fin de rampa 
                    flag_desa = False
            if not flag_acel and not flag_const and not flag_desa:
                state = 3

            pilot_node.u_action[1] = x_speed #-----> Y speed
            pilot_node.publish3()






            # error_x = x_localp - pilot_node.localpos.pose.pose.position.x
            # error_y = y_localp - pilot_node.localpos.pose.pose.position.y

            # error_x = math.pow(error_x,2)
            # error_y = math.pow(error_y,2)
            # error = math.sqrt(error_x + error_y)
            # if error <= 1:
            #     print('punto alcanzado')
            #     rospy.sleep(2.)
            #     state = 3
            # else:
            #     pilot_node.getpose[0] = x_localp
            #     pilot_node.getpose[1] = y_localp
            #     pilot_node.getpose[2] = takeoff_altitude

            #     pilot_node.publish()

        elif state3:
            new_state_alert(state)
            if flag_alinea1:

                #------- Ejecutando control en eje x--------
                x_control.controller_pv = pilot_node.process_value[0]
                x_control.process()
                print('Alineandose con pad')
                pilot_node.u_action[0]=x_control.output

                #---- Ejecutando control en eje y ---------
                y_control.controller_pv = pilot_node.process_value[1]
                y_control.process()
                pilot_node.u_action[1]= -y_control.output


                if x_control.flag and y_control.flag and (x_control.action_flag or y_control.action_flag):
                    rospy.sleep(5.)
                    x_control.flag = False
                    y_control.flag = False
                    x_control.action_flag = False
                    y_control.action_flag = False

                if x_control.error<=20.0 and x_control.error>=-20.0 and y_control.error<=20.0 and y_control.error>=-20.0:
                    flag_alinea1 = False
                    flag_decenso = True

            if flag_decenso:
                if pilot_node.altitude >= 7.0:
                    pilot_node.u_action = [0.0, 0.0, 0.0 , -1.0]
                else:
                    pilot_node.u_action = [0.0, 0.0, 0.0, 0.0]
                    flag_decenso = False
                    flag_alinea2 = True
            if flag_alinea2:
                #------- Ejecutando control en eje x--------
                x_control.controller_pv = pilot_node.process_value[0]
                x_control.process()
                print('Alineandose con pad')
                pilot_node.u_action[0]=x_control.output

                #---- Ejecutando control en eje y ---------
                y_control.controller_pv = pilot_node.process_value[1]
                y_control.process()
                pilot_node.u_action[1]= -y_control.output


                if x_control.flag and y_control.flag and (x_control.action_flag or y_control.action_flag):
                    rospy.sleep(5.)
                    x_control.flag = False
                    y_control.flag = False
                    x_control.action_flag = False
                    y_control.action_flag = False

                if x_control.error<=10.0 and x_control.error>=-10.0 and y_control.error<=10.0 and y_control.error>=-10.0:
                    flag_alinea2 = False
                    flag_decenso = False
                    pilot_node.u_action = [0.0, 0.0, 0.0, 0.0]
                    pilot_node.pilot_mode(custom_mode='LAND')

                    

            pilot_node.publish3()
            

        pilot_node.state.data = state
        pilot_node.publish2()
        rate.sleep()

       



