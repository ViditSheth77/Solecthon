import cv2
import numpy as np
import math

path = "http://192.168.43.156:4747/video"

# inv map output-image size
img_dim = (416, 285) # (w, h) = (x, y)

# sampeling speed
BAUD_RATE = 115200

# intel camera 
#pt = [(0,225), (-1500,500), (600,225), (2100,500)]

# Laptop camera 
pt_in = [(0   , 100),
         (-600, 416),
         (416 , 100), 
         (1016, 416)]

pt_out = [(0         , 0         ),
          (0         , img_dim[1]),
          (img_dim[0], 0         ), 
          (img_dim[0], img_dim[1])]

# threshold after which detections won't be considered
# below variable represents threshold 'y' coordinate
LIMIT_CONE = 150 

# when one side is empty of cones, this variable is used as offset
mid_c = 80-5

# car coordinates on image
car_coor = (img_dim[0]//2, img_dim[1]+25)

def angle(p1, p2):
    x, y = p1
    p, q = p2
    try:
        slope = (q - y)/(p - x)
    except:
        slope = 99999
    angle = np.arctan(slope)*180/math.pi
    if(angle > 0):
        return -1*(90 - angle)
    return (90 + angle)

def inv_map(frame):
    pts1 = np.float32(pt_in)
    pts2 = np.float32(pt_out)
    M = cv2.getPerspectiveTransform(pts1,pts2)
    image = cv2.warpPerspective(frame, M, img_dim, flags=cv2.INTER_LINEAR)
    #cv2.imshow('itshouldlookfine!', image)
    return image, M

def inv_coor(bounding_rects, M, image):
    mybox = []
    for detection in bounding_rects:

        xmax = detection[0]
        xmin = detection[1]
        ymax = detection[2]
        ymin = detection[3]
        #print( ((xmax+xmin)//2), (ymax) )
        pt1 = (int(xmin), int(ymin))
        pt2 = (int(xmax), int(ymax))
        cv2.circle(image,pt1, 5, (255,255,255), -1)
        cv2.circle(image,pt2, 5, (255,255,255), -1)
    #for rect in bounding_rects:
        a = np.array([[( (xmax+xmin)//2 ), (ymax//1)]], dtype='float32')
        a = np.array([a])
        pointsOut = cv2.perspectiveTransform(a, M)
        box = pointsOut[0][0][0], pointsOut[0][0][1]
        mybox.append(box)
        #print(pointsOut)
    #mybox = sorted(mybox, key=lambda k:(k[1], k[0])).copy()
    #mybox.reverse()
    #abc = sorted(mybox, key=last)
    print('boxall', mybox)
    return mybox , image

def st_line( a, b, c, x, y ):
    if( a*x + b*y + c < 0 ):
        return True# True means left side for left turn
    return False

def line_x(direction_coor, cone_coor):
    car_x, car_y = car_coor
    cone_x, cone_y = cone_coor
    direction_x, direction_y = direction_coor

    slope = (direction_y - car_y) / (direction_x - car_x)

    x_on_line = slope * (cone_y - car_y) - car_x

    error = cone_x - x_on_line

    if(error >= 0):
        # cone on right side
        # True indicates right side
        return True
    else:
        # cone on left side
        # False indicates left side
        return False

def pathplan(mybox, str_ang):
    left_box = []
    right_box = []
    left_count = 5
    right_count = 5
    ratio = 0.25

    for i in range(len(mybox)):
        x, y = mybox[i]
        if( str_ang == '3' or str_ang == '4' or  str_ang == '5' ):
            if(x < 208):
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1

            else:
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

        elif( str_ang == '0' or str_ang == '1' or str_ang == '2'):
            if( not line_x( (img_dim[0]*ratio, 0), (x, y) ) ):
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1
            else:
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

        elif( str_ang == '6' or str_ang == '7' or str_ang == '8' ):
            if( line_x( ( img_dim[0] - img_dim[0]*ratio, 0), (x, y) ) ):
                if(right_count > 0):
                    right_box.append(mybox[i])
                    right_count = right_count - 1

            else:
                if(left_count > 0):
                    left_box.append(mybox[i])
                    left_count = left_count - 1


	
    #############################################################################
    left_box.sort(reverse = True)
    right_box.sort(reverse = True)

    left_box =  sorted(left_box, key=lambda k:(k[1], k[0])).copy()
    right_box = sorted(right_box, key=lambda l:(l[1], l[0])).copy()
    '''left_box.sort()
    right_box.sort()'''
    #############################################################################
    ############################### path planning ###############################
    #############################################################################
    try:
        if(left_box[-1][1] < LIMIT_CONE):
            left_box.clear()
    except:
        print('Left Exception in pathplan function.............')
            
    try:
        if(right_box[-1][1] < LIMIT_CONE):
            right_box.clear()
    except:
        print('Right Exception in pathplan function.............')
    #############################################################################
    
    lines = []
    lines.append(car_coor)


    if( len(left_box) == 0 and len(right_box) == 0 ):
        lines.append((208,350))
         
    elif( len(left_box) == 0 and len(right_box) != 0 ):
        for i in range(len(right_box)):
            #print( 'test1' )
            x, y = right_box[i]
            x = x - mid_c
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) == 0 ):
        for i in range(len(left_box)):
            #print( 'test2' )
            x, y = left_box[i]
            x = x + mid_c
            lines.append( (int(x), int(y)) )
        
    elif( len(left_box) != 0 and len(right_box) != 0 ):

        small_len  = 0
        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()
        if(len(left_box) > len(right_box)):
            small_len = len(right_box)
        else:
            small_len = len(left_box)
        
        for i in reversed(range(small_len)):
                #print( 'test3' )
                x, y = tuple(np.add((right_box[i]), (left_box[i])))
                x = x//2
                y = y//2
                #cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1) 	# Filled
                lines.append( (int(x), int(y)) )

        left_box = left_box[::-1].copy()
        right_box = right_box[::-1].copy()

    lines = sorted(lines, key=lambda m:(m[1], m[0])).copy()
    #print(len(left_box), len(right_box))
    
    return left_box[::-1], right_box[::-1], lines[::-1]

def pathbana(lines, inv_image):
    for i in range(len(lines) - 1):
        cv2.circle(inv_image,lines[i], 5, (0,0,0), -1) 	# Filled
        #print( 'test4' )
        inv_image = cv2.line(inv_image,lines[i],lines[i+1],(255,255,0),4)
    '''if(angle(lines[0], lines[1]) > 75 or angle(lines[0], lines[1]) < -75):
        lines.remove(1)'''
	
    #print( lines[0], lines[1] , angle(lines[0], lines[1]) )

    return inv_image