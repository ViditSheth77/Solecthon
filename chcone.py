import cv2
import numpy as np
import math

path = "http://192.168.43.156:4747/video"
cap = cv2.VideoCapture('video.mp4')

# Laptop camera 
pt = [(0,100), (-900,450), (600,100), (1500,450)]

# intel camera 
#pt = [(0,225), (-1500,500), (600,225), (2100,500)]

def angle(p1, p2):
    x, y = p1
    p, q = p2
    try:
        slope = (q - y)/(p - x)
    except:
        slope = 99999
    angle = np.arctan(slope)*180/math.pi
    if(angle > 0):
        return(90 - angle)
    return -1*(90 + angle)

while True:

    #############################################################################
    ##########################  cone detection  #################################
    #############################################################################
    _, frame = cap.read()
    #frame = cv2.imread('coneimg.png')
    frame = cv2.resize(frame, (600, 450))
    img_HSV = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    img_thresh_low = cv2.inRange(img_HSV, np.array([0, 135, 135]),np.array([15, 255, 255]))  # everything that is included in the "left red"

    img_thresh_high = cv2.inRange(img_HSV, np.array([159, 135, 135]), np.array([179, 255, 255]))  # everything that is included in the "right red"
                                 
    img_thresh_mid = cv2.inRange(img_HSV, np.array([100, 150, 0]),np.array([140, 255, 255]))  # everything that is included in the "right red"
                                 
    img_thresh = cv2.bitwise_or(img_thresh_low, img_thresh_mid)  # combine the resulting image
    img_thresh = cv2.bitwise_or(img_thresh, img_thresh_high)
    kernel = np.ones((5, 5))
    img_thresh_opened = cv2.morphologyEx(img_thresh, cv2.MORPH_OPEN, kernel)
    img_thresh_blurred = cv2.medianBlur(img_thresh_opened, 5)
    img_edges = cv2.Canny(img_thresh_blurred, 80, 160)
    contours, _ = cv2.findContours(np.array(img_edges), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    img_contours = np.zeros_like(img_edges)
    cv2.drawContours(img_contours, contours, -1, (255, 255, 255), 2)
    approx_contours = []

    for c in contours:
        approx = cv2.approxPolyDP(c, 10, closed=True)
        approx_contours.append(approx)
    img_approx_contours = np.zeros_like(img_edges)
    cv2.drawContours(img_approx_contours, approx_contours, -1, (255, 255, 255), 1)
    all_convex_hulls = []
    for ac in approx_contours:
        all_convex_hulls.append(cv2.convexHull(ac))
    img_all_convex_hulls = np.zeros_like(img_edges)
    cv2.drawContours(img_all_convex_hulls, all_convex_hulls, -1, (255, 255, 255), 2)
    convex_hulls_3to10 = []
    for ch in all_convex_hulls:
        if 3 <= len(ch) <= 10:
            convex_hulls_3to10.append(cv2.convexHull(ch))
    img_convex_hulls_3to10 = np.zeros_like(img_edges)
    cv2.drawContours(img_convex_hulls_3to10, convex_hulls_3to10, -1, (255, 255, 255), 2)


    def convex_hull_pointing_up(ch):
        '''Determines if the path is directed up.
        If so, then this is a cone. '''

        # contour points above center and below

        points_above_center, points_below_center = [], []

        x, y, w, h = cv2.boundingRect(ch)  # coordinates of the upper left corner of the describing rectangle, width and height
        aspect_ratio = w / h  # ratio of rectangle width to height

        # if the rectangle is narrow, continue the definition. If not, the circuit is not suitable
        if aspect_ratio < 0.8:
	# We classify each point of the contour as lying above or below the center	
            vertical_center = y + h / 2

            for point in ch:
                if point[0][
                    1] < vertical_center:  # if the y coordinate of the point is above the center, then add this point to the list of points above the center
                    points_above_center.append(point)
                elif point[0][1] >= vertical_center:
                    points_below_center.append(point)

            # determine the x coordinates of the extreme points below the center
            left_x = points_below_center[0][0][0]
            right_x = points_below_center[0][0][0]
            for point in points_below_center:
                if point[0][0] < left_x:
                    left_x = point[0][0]
                if point[0][0] > right_x:
                    right_x = point[0][0]

            # check if the upper points of the contour lie outside the "base". If yes, then the circuit does not fit
            for point in points_above_center:
                if (point[0][0] < left_x) or (point[0][0] > right_x):
                    return False
        else:
            return False

        return True


    cones = []
    bounding_rects = []
    for ch in convex_hulls_3to10:
        if convex_hull_pointing_up(ch):
            cones.append(ch)
            rect = cv2.boundingRect(ch)
            bounding_rects.append(rect)
    img_res = frame.copy()
    cv2.drawContours(img_res, cones, -1, (255, 255, 255), 2)
    transf = np.zeros([450, 600, 3])

    mybox = []
    pts1 = np.float32([pt[0],pt[1],pt[2],pt[3]])
    pts2 = np.float32([[0,0],[0,450],[600,0],[600,450]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    transf = np.zeros([450, 600, 3])
    dst2 = cv2.warpPerspective(img_res,M,(600,450), flags=cv2.INTER_LINEAR)


    for rect in bounding_rects:
        #print('previous', rect[0], rect[1], rect[2], rect[3])
        cv2.rectangle(img_res, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (1, 255, 1), 6)
        cv2.circle(img_res,(rect[0], rect[1]), 5, (0,200,255), -1)
        cv2.circle(img_res,(rect[0] + rect[2], rect[1] + rect[3]), 5, (0,200,255), -1)
        cv2.circle(img_res,(rect[0] + rect[2]//2, rect[1] + rect[3]), 5, (255,255,255), -1)
        a = np.array([[(rect[0] + rect[2]//2), (rect[1] + rect[3])]], dtype='float32')
        a = np.array([a])

        pointsOut = cv2.perspectiveTransform(a, M)
        box = pointsOut[0][0][0], pointsOut[0][0][1]
        mybox.append(box)
        cv2.circle(img_res,((rect[0] + rect[2]//2), (rect[1] + rect[3])), 5, (0,0,255), -1)
        cv2.circle(dst2,box, 5, (0,225,255), -1)
        cv2.circle(transf,box, 5, (0,225,255), -1)



    #############################################################################



    #############################################################################
    ####################### inverse perspective transform   #####################
    #############################################################################

    img = cv2.resize(img_res, (600, 450))
    rows,cols,channels = img.shape

    #cv2.circle(transf,pt[0], 5, (0,0,255), -1) 	# Filled
    #cv2.circle(transf,pt[1], 5, (0,0,255), -1) 	# Filled
    #cv2.circle(transf,pt[2], 5, (0,0,255), -1) 	# Filled
    #scv2.circle(transf,pt[3], 5, (0,0,255), -1) 	# Filled

    cv2.circle(img,pt[0], 5, (0,0,255), -1) 	# Filled
    cv2.circle(img,pt[1], 5, (0,0,255), -1) 	# Filled
    cv2.circle(img,pt[2], 5, (0,0,255), -1) 	# Filled
    cv2.circle(img,pt[3], 5, (0,0,255), -1) 	# Filled

	#pts1 = np.float32([[30,111],[34,326],[561,53],[554,381]])

    dst = cv2.warpPerspective(img,M,(600,450), flags=cv2.INTER_LINEAR)

    left_box = []
    right_box = []

    for i in range(len(mybox)):
        x, y = mybox[i]
        if(x < 300):
            left_box.append(mybox[i])
        else:
            right_box.append(mybox[i])

    '''for i in range(len(left_box) - 1):
        cv2.circle(transf,left_box[0], 5, (0,0,255), -1) 	# Filled
        dst2 = cv2.line(dst2,left_box[i],left_box[i+1],(255,0,0),5)

    for i in range(len(right_box) - 1):
        cv2.circle(transf,right_box[0], 5, (0,0,255), -1) 	# Filled
        dst2 = cv2.line(dst2,right_box[i],right_box[i+1],(255,0,0),5)'''

    #############################################################################

    #############################################################################
    ####################### path planning   #####################################
    #############################################################################

     #############################################################################
    
    lines = []
    lines.append((300,500))

    mid_c = 100

    if( len(left_box) == 0 and len(right_box) == 0 ):
        lines.append((300,400))
         
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

        if(len(left_box) > len(right_box)):
            small_len = len(right_box)
        else:
            small_len = len(left_box)
        
        for i in range(small_len):
                #print( 'test3' )
                x, y = tuple(np.add((right_box[i]), (left_box[i])))
                x = x//2
                y = y//2
                cv2.circle(transf,(int(x), int(y)), 5, (255,0,255), -1) 	# Filled
                lines.append( (int(x), int(y)) )

        '''if(len(left_box) > len(right_box)):
            i = len(right_box) + 1
            while(i != len(left_box)):
                x, y = left_box[i]
                x = x + mid_c
                lines.append( (int(x), int(y)) )
                i = i + 1
                transf = cv2.line(transf,(0,0),lines[-1],(255,0,0),5)

        elif( len(left_box) < len(right_box) ):
            i = len(left_box) + 1
            print('i am in')
            while(i != len(right_box)):
                x, y = right_box[i]
                x = x - mid_c
                lines.append( (int(x), int(y)) )
                i = i + 1
                transf = cv2.line(transf,(0,0),lines[-1],(255,0,0),5)'''

    for i in range(len(lines) - 1):
        #cv2.circle(transf,lines[0], 5, (255,255,0), -1) 	# Filled
        #print( 'test4' )
        dst2 = cv2.line(dst2,lines[i],lines[i+1],(255,255,0),5)
        transf = cv2.line(transf,lines[i],lines[i+1],(255,255,0),5)        

    print( lines[0], lines[1] , angle(lines[0], lines[1]) )


    cv2.imshow('image',img_res)
    cv2.imshow('transform', dst2)
    cv2.imshow('coordinates', transf)

    # clear lists
    mybox.clear()
    left_box.clear()
    right_box.clear()
    lines.clear()

    #############################################################################

    key = cv2.waitKey(50)
    if key == 27:
        break

## Close and exit
cap.release()
#out.release()
cv2.destroyAllWindows()