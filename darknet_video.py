from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet
import chcone
import serial

DANGER = 150

a = range(-90,-75)
b = range(-75,-60)
c = range(-60,-45)
d = range(-45,-30)
e = range(-30,-15)
f = range(-15,0)
g = range(0,15)
h = range(15,30)
i = range(30,45)
j = range(45,60)
k = range(60,75)
l = range(75,90)

m = range(-90,-30)
n = range(-30,-12)
o = range(-12,0)
p = range(0,12)
q = range(12,30)
r = range(30,90)

s = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(1.5)


def steer(angle):
    '''if( angle in a ):
        return '0'
    elif( angle in b ):
        return '1'
    elif( angle in c ):
        return '2'
    elif( angle in d ):
        return '2'
    elif( angle in e ):
        return '3'
    elif( angle in f ):
        return '4'
    elif( angle in g ):
        return '4'
    elif( angle in h ):
        return '5'
    elif( angle in i ):
        return '6'
    elif( angle in j ):
        return '7'
    elif( angle in k ):
        return '8'
    elif( angle in l):
        return '9'
    '''
    if( angle in m ):
        return '0'
    elif( angle in n ):
        return '1'
    elif( angle in o or angle in p):
        return '2'
    elif( angle in q):
        return '3'
    elif( angle in r):
        return '4'
    return '2'
    '''else:
        print("OUT OF ANGLE!!!")'''

def personDistance(person_coor):
	p_x, p_y = person_coor
	c_x, c_y = chcone.car_coor
	distance = math.sqrt( (p_x - c_x)**2 + (p_y - c_y)**2 )
	return distance

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        cv2.circle(img,pt1, 5, (255,255,255), -1)
        '''cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
        cv2.putText(img,
                    detection[0].decode() +
                    " [" + str(round(detection[1] * 100, 2)) + "]",
                    (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    [0, 255, 0], 2)'''
    return img
    
    
def get_inv_coor(detections, img, M):
    mybox = []
    person = []
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        xmin, ymin, xmax, ymax = convertBack(
            float(x), float(y), float(w), float(h))
        pt1 = (xmin, ymin)
        pt2 = (xmax, ymax)
        #print(type(detection[0]))
        #person.append( ( (xmin+xmax)//2,(ymax) ) )
        cv2.circle(img,pt1, 5, (255,255,255), -1)
        a = np.array([[( (xmax+xmin)//2 ), (ymax//1)]], dtype='float32')
        a = np.array([a])
        pointsOut = cv2.perspectiveTransform(a, M)
        box = pointsOut[0][0][0], pointsOut[0][0][1]
        if(detection[0].decode('utf-8') == 'person'):
        	person.append(box)
        elif(box[1]>0):
        	mybox.append(box)
    
    sorted(mybox, key=lambda k:(k[1], k[0]))

    #print(mybox[::-1],'\n')

    return person, mybox[::-1], img



netMain = None
metaMain = None
altNames = None


def YOLO():

    global metaMain, netMain, altNames
    configPath = "./tiny9000.cfg"
    weightPath = "./tiny_10000.weights"
    metaPath = "./data/obj.data"
    if not os.path.exists(configPath):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
    if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
    if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")
    if netMain is None:
        netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
    if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))
    if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass
    path = 'http://192.168.43.156:4747/video'
    #cap = cv2.VideoCapture(path)
    cap = cv2.VideoCapture(3)
    #cap = cv2.VideoCapture('video143.mp4')
    cap.set(3, 1280)
    cap.set(4, 720)
    out = cv2.VideoWriter(
        "output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
        (darknet.network_width(netMain), darknet.network_height(netMain)))
    print("Starting the YOLO loop...")

    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)
        
    #s.write(str.encode('a'))
    
    counter = 0
    sterring = '2'

    while cap.isOpened():
        try:
            prev_time = time.time()
            ret, frame_read = cap.read()
            frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
            frame_resized = cv2.resize(frame_rgb,
                                       (darknet.network_width(netMain),
                                        darknet.network_height(netMain)),
                                       interpolation=cv2.INTER_LINEAR)
            darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())

            detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.25)
            image = cvDrawBoxes(detections, frame_resized)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            #print(1/(time.time()-prev_time))

            ######################################################
            #############   AAPNA CODE  ##########################
            ######################################################

            # simple inv transform
            inv_image, M = chcone.inv_map(image)
                
            # getting inv coordinates on person and cone
            person, mybox, image = get_inv_coor(detections, image, M)

            # alert in case of threat
            if(len(person) != 0):
            	for human in person:
            		cv2.circle(inv_image, human, 5, (0,0,255), -1)
            		distance = personDistance(human)
            		'''if(distance <= DANGER):
            			print("U wanna Die?")
            		else:
            			print("hatt be")'''
            

            left_box, right_box, lines = chcone.pathplan(mybox)
            #print(left_box, '\n')
            #print(right_box,'\n')
            #print(lines,'\n\n\n')
            if len(mybox) == 0:
                counter = counter + 1
                if counter == 27:
                    s.write(str.encode('c'))
                    counter = 0



            ######################################################
            ############### DRAWING ONLY    ######################
            ######################################################

            for i in range(len(mybox)):
                cv2.circle(inv_image, mybox[i], 5, (0,255,255), -1)   # Filled

            for i in range(len(left_box)-1):
                cv2.line(inv_image, left_box[i], left_box[i+1], (125, 125, 255), 3)

            for i in range(len(right_box)-1):
                cv2.line(inv_image, right_box[i], right_box[i+1], (0,0,0), 3)

            ######################################################
            ######################################################




            # encode signal for steering control
            try:
            	angle = chcone.angle(lines[0], lines[1])
            except:
            	angle = chcone.angle(lines[0], lines[1])
            angle = math.floor(angle)
            angle_a = steer(angle)
            print(angle)
            if(sterring != angle_a):
                s.write(str.encode(angle_a))
                steering = angle_a

            # JUST DRAWING
            inv_image = chcone.pathbana(lines, inv_image)
            cv2.circle(image,chcone.pt[0], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[1], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[2], 5, (255,255,255), -1)
            cv2.circle(image,chcone.pt[3], 5, (255,255,255), -1)
            image = cv2.resize(image, (800, 800))
            cv2.imshow('image', image)
            
            cv2.circle(inv_image, chcone.car_coor, DANGER, (0,0,225), 2)
            h, w, c = inv_image.shape
            #draws center line
            #cv2.line(inv_image, (w//2,0), (w//2,h),(255,0,0),5)
            cv2.line(inv_image, (0, chcone.LIMIT_CONE), (416, chcone.LIMIT_CONE), (0,0,255), 1)

            inv_image = cv2.resize(inv_image, (800, 800))    

            cv2.imshow('transform', inv_image)
        
                # clear lists
            mybox.clear()
            left_box.clear()
            right_box.clear()
            lines.clear()
            
    	
            cv2.waitKey(3)
        except Exception as e:
            print(e)
            print('Exception aaya hai!!!!')
            s.write(str.encode('c'))
            break
    cap.release()
    out.release()
if __name__ == "__main__":
    YOLO()
