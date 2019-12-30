import cv2
import time
import numpy as np
#path = "http://192.168.1.202:4747/video"
cap = cv2.VideoCapture('video.mp4')
frame_width = int(600)
frame_height = int(450)
# Laptop camera 
pt = [(0,225), (-1100,500), (600,225), (1700,500)]

# intel camera 
#pt = [(0,225), (-1500,500), (600,225), (2100,500)]

def convex_hull_pointing_up(ch):
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

while True:

    #############################################################################
    ##########################  cone detection  #################################
    #############################################################################
    _, frame = cap.read()
    #frame = cv2.imread('coneimg.png')
    frame = cv2.resize(frame, (600, 450))
    start_time = time.time()
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


    for rect in bounding_rects:
        cv2.rectangle(img_res, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (1, 255, 1), 6)
        #print(rect[0], rect[1], rect[2], rect[3])
        #cv2.circle(transf,(150, 100), 5, (0,0,255), -1)
        #cv2.circle(transf,((rect[0] + rect[2])//1, (rect[1] + rect[3])//1), 5, (0,0,255), -1)
        #cv2.circle(transf,((rect[0] + rect[2])//1, (rect[1] + rect[3])//1), 5, (0,0,255), -1)
    #cv2.imshow("Red", img_res)
    #cv2.imshow('seeing', img_thresh)


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
    pts1 = np.float32([pt[0],pt[1],pt[2],pt[3]])
    pts2 = np.float32([[0,0],[0,450],[600,0],[600,450]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    N = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(img,M,(600,450), flags=cv2.INTER_LINEAR)
    cv2.imshow('image',img)
    dst = cv2.resize(dst, (600, 450))

    mybox = []
    left_box = []
    right_box = []

    for rect in bounding_rects:

        # provide a point you wish to map from image 1 to image 2
        a = np.array([[(rect[0] + rect[2]), (rect[1] + rect[3])]], dtype='float32')
        a = np.array([a])

        # finally, get the mapping
        pointsOut = cv2.perspectiveTransform(a, N)
        box = pointsOut[0][0][0], pointsOut[0][0][1]
        print(box)
        mybox.append(box)
        cv2.circle(transf,box, 5, (0,0,255), -1)

    for i in range(len(mybox)):
        x, y = mybox[i]
        if(x < 300):
            left_box.append(mybox[i])
            #transf = cv2.line(transf,(0,0),mybox[i],(255,0,0),5)
        else:
            right_box.append(mybox[i])
            #transf = cv2.line(transf,(0,0),mybox[i],(255,0,0),5)

    # Only for visual purpose
    for i in range(len(left_box) - 1):
        transf = cv2.line(transf,left_box[i],left_box[i+1],(255,0,0),5)
    # Only for visual purpose
    for i in range(len(right_box) - 1):
        transf = cv2.line(transf,right_box[i],right_box[i+1],(255,0,0),5)

 #   for i in range(len(right_box) - 1):
  #      transf = cv2.line(transf,right_box[i]+left_box[i],right_box[i+1]+left_box[i+1],(255,0,0),5)    

    

    cv2.imshow("coordinates Real??", transf)
    cv2.imshow('transform', dst)
    #print(str(len(bounding_rects)) + ' cone(s) found in the picture')
    #print("--- %s seconds ---" % (time.time() - start_time))
    mybox.clear()
    left_box.clear()
    right_box.clear()

    #############################################################################

    start_time = time.time()
    key = cv2.waitKey(1)
    if key == 27:
        break

## Close and exit
cap.release()
cv2.destroyAllWindows()
