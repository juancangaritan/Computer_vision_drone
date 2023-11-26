#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import time
import onnxruntime as ort
import random
import numpy as np


## ------------ Frame capture -----------

class dnn_node:
    def __init__(self):
        self.node_name = 'dnn_detector'
        self.image_topic = '/roscam/cam/image_raw'
        self.dnn_path = '/home/juancangaritan/yolov7/best_aterrizaje.onnx'
        self.cuda = True
        self.bridge = CvBridge()
        self.init_node()
        self.pub = rospy.Publisher('detector_node',Float32MultiArray,queue_size=10)
        # --- dnn atributes ----

        self.new_shape = (640, 640)
        self.color = (114, 114, 114)
        self.auto = False
        self.scaleup = True
        self.stride = 32
        self.time_ini = 0
        self.time_fin = 0
        self.fps = 0
        self.seconds = 0.1
        self.aterrizaje_pad = 0
        self.frame_refresh = 0

        #----- initial variables ------
        self.delay = True
    
    def init_node(self):

        rospy.init_node(self.node_name,anonymous=False)
        rospy.Subscriber(self.image_topic, data_class= Image, callback= self.frame_callback)
        self.providers = ['CUDAExecutionProvider' , 'CPUExecutionProvider'] if self.cuda else ['CPUExecutionProvider']
        self.session = ort.InferenceSession(self.dnn_path, providers = self.providers)

        print('iniciando red neuronal..')
    
    def frame_callback(self,data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        


    def dnn_loop(self,arg):
        print(arg)
        print('Ejecutando callback')
        #cv2.imshow('test',self.img)
        #cv2.waitKey(1)

        self.pre_detect()
        self.letterbox()
        self.detect()


    #----------- metodos onnx ------------

    def letterbox(self):
        self.new_shape = (640, 640)
        self.im = self.img.copy()
        self.shape = self.im.shape[:2] #Forma actual [alto,ancho]
        if isinstance(self.new_shape, int):
            self.new_shape = (self.new_shape, self.new_shape)

        # ratio de escala(nuevo/viejo)
        self.r = min(self.new_shape[0]/self.shape[0], self.new_shape[1]/self.shape[1])
        #print(self.r)
        if not self.scaleup:
            self.r = min(self.r, 1.0)

        #padding
        new_unpad = int(round(self.shape[1]*self.r)),int(round(self.shape[0]*self.r))
        #print(new_unpad)
        self.dw, self.dh = self.new_shape[1] - new_unpad[0], self.new_shape[0] - new_unpad[1]
        #print(self.dw)
        #print(self.dh)
        if self.auto:
            self.dw, self.dh = np.mod(self.dw, self.stride), np.mod(self.dh, self.stride)
            
        self.dw /= 2
        self.dh /= 2

        if self.shape[::-1] != new_unpad: 
            self.im = cv2.resize(self.im, new_unpad, interpolation=cv2.INTER_LINEAR)



        top, bottom = int(round(self.dh - 0.1)), int(round(self.dh + 0.1))
        #print(top)
        left, right = int(round(self.dw - 0.1)), int(round(self.dw + 0.1))
        #print('top = {} \n bottom = {} \n left = {} \n right = {}'.format(top,bottom,left,right))
        self.im = cv2.copyMakeBorder(self.im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=self.color)

    def pre_detect(self):
        self.names = ['Aterrizaje_pad', 'Master_pad']
        self.img2 = self.img.copy()
        self.colors={name:[random.randint(0, 255) for _ in range(3)] for i,name in enumerate(self.names)}
    def detect(self):
        image = self.im
        ratio = self.r
        dwdh = (self.dw, self.dh)
        image = image.transpose((2, 0, 1))
        image = np.expand_dims(image,0)
        image = np.ascontiguousarray(image)
        im = image.astype(np.float32)
        im /= 255
        im.shape
        outname = [i.name for i in self.session.get_outputs()]
        outname

        inname = [i.name for i in self.session.get_inputs()]
        inname

        inp = {inname[0]:im}

		# Inferencia ONNX (Conclusion ONNX)
        verde=[51,255,71]
        blanco=[255,255,255]
        outputs = self.session.run(outname, inp)[0]
        if self.frame_refresh < 6:
            self.frame_refresh += 1
        else:
            self.frame_refresh = 0
        
        if len(outputs) == 0:
            if self.frame_refresh == 0:
                self.aterrizaje_pad = 0
            ori_image = self.img.copy()
            print('\n \n \n \nSin detecciones')
            overlay2 = ori_image.copy()
            
            data_rect0 = (0,0)
            data_rect1 = (230,150)
            text_org = (15, 15)
            cv2.rectangle(overlay2, data_rect0,data_rect1,[20,20,20],-1)
            cv2.putText(ori_image,'Aterrizaje de precision' ,(text_org[0],text_org[1]+15*0), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)
            cv2.putText(ori_image,'FPS = {}'.format(self.fps) ,(text_org[0],text_org[1]+15*1), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)
            cv2.putText(ori_image,'Aterrizaje pad = {}'.format(self.aterrizaje_pad) ,(text_org[0],text_org[1]+15*2), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)

            self.test = cv2.addWeighted(overlay2,0.3,ori_image,1-0.3,0,ori_image)
            cv2.imshow('drone view', self.test)
            cv2.waitKey(1)
        #print(len(outputs))
		#outputs
        
        ori_images = [self.img.copy()]
        for i,(batch_id,x0,y0,x1,y1,cls_id,score) in enumerate(outputs):
            if self.frame_refresh == 0:
                self.aterrizaje_pad = np.count_nonzero(cls_id==0)
            aterrizaje = cls_id == 0
            if aterrizaje :
                self.pos = [(x0 + (x1-x0)/2) , (y0 + (y1-y0)/2) ]
                print(self.pos)
            
            print('\nBatch ID: {} \nClase: {} \nPuntaje: {}'.format(batch_id,cls_id,score))
            image = ori_images[int(batch_id)]
            
            box = np.array([x0,y0,x1,y1])
            box -= np.array(dwdh*2)
            box /= ratio
            box = box.round().astype(np.int32).tolist()
            cls_id = int(cls_id)
            #print(cls_id)
            score = round(float(score),3)
            #print(score)
            name = self.names[cls_id]
            color = self.colors[name]
            name += ' '+str(score)
            #print(box[:2])
            #print(box[2:])
            punto_ini=tuple(box[:2])
            punto_fin=tuple(box[2:])
            cv2.rectangle(image,punto_ini,punto_fin,[255,71,51],2)
            cv2.putText(image,name,(box[0], box[1] -15), cv2.FONT_HERSHEY_DUPLEX,0.99,[255, 71,51], thickness=1)
            overlay = image.copy()
            cv2.rectangle(overlay, punto_ini, punto_fin,[255,71,51],-1)
            cv2.addWeighted(overlay,0.3,image,1-0.3,0,image)
            overlay2 = image.copy()
            
            data_rect0 = (0,0)
            data_rect1 = (230,150)
            text_org = (15, 15)
            cv2.rectangle(overlay2, data_rect0,data_rect1,[20,20,20],-1)
            cv2.putText(image,'Aterrizaje de precision' ,(text_org[0],text_org[1]+15*0), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)
            cv2.putText(image,'FPS = {}'.format(self.fps) ,(text_org[0],text_org[1]+15*1), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)
            cv2.putText(image,'Aterrizaje pad = {}'.format(self.aterrizaje_pad) ,(text_org[0],text_org[1]+15*2), cv2.FONT_HERSHEY_COMPLEX,0.5,verde, thickness=1)
            self.test = cv2.addWeighted(overlay2,0.3,image,1-0.3,0,image)
            if int(round(self.dw)) == 0:
                cv2.imshow('drone view',self.test)
                cv2.waitKey(1)
            else:
                cv2.imshow('drone view',self.img)
            msg = Float32MultiArray()
            msg.data = self.pos
            self.pub.publish(msg)
            

        self.time_fin =time.time()
        self.seconds = self.time_fin - self.time_ini
        self.fps = int(round(1/self.seconds))

if __name__ == '__main__':

    pad_detect = dnn_node()
    try:
        rospy.Timer(rospy.Duration(0.1), pad_detect.dnn_loop)
        rospy.spin()
        
    except:
        print('error')


    


        

    


