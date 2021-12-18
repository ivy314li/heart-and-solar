import cv2
import numpy as np
import pathlib
import os
import cv2.aruco as aruco
import argparse
import sys
import imutils
import math
import time
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import statistics


""" certain functions were sourced from the following online resources
 https://medium.com/vacatronics/3-ways-to-calibrate-your-camera-using-opencv-and-python-395528a51615
 https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
 https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/blob/main/pose_estimation.py 
"""



def save_coefficients(mtx, dist, path):
    '''Save the camera matrix and the distortion coefficients to given path/file.'''
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    cv_file.write('K', mtx)
    cv_file.write('D', dist)
    # note you *release* you don't close() a FileStorage object
    cv_file.release()

def load_coefficients(path):
    '''Loads camera matrix and distortion coefficients.'''
    # FILE_STORAGE_READ
    cv_file = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)

    # note we also have to specify the type to retrieve other wise we only get a
    # FileNode object back instead of a matrix
    camera_matrix = cv_file.getNode('K').mat()
    dist_matrix = cv_file.getNode('D').mat()

    cv_file.release()
    return [camera_matrix, dist_matrix]

"""
finds center of a marker
"""
def find_center(markerCorner):
    (topLeft, topRight, bottomRight, bottomLeft) = markerCorner[0]

    # convert each of the (x, y)-coordinate pairs to integers
    topRight = (int(topRight[0]), int(topRight[1]))
    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
    topLeft = (int(topLeft[0]), int(topLeft[1]))

    # find center marker
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    return cX, cY

"""
simple distance formula
"""
def find_distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
"""
helper for intersection of 2 circles
"""
def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # non intersecting
    if d > r0 + r1 :
        return None
    # One circle within other
    if d < abs(r0-r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d
        
        return [x3, y3], [x4, y4]
    
def diff_y_angle(rvec_list):    
    og_angle = 0
    try:
        for i in range(4):
            counts += 1
            og_angle += rvec_list[i][1]
    except:
        og_angle = 0
    
    diff = 180* (rvec_list[4][1]-og_angle)/math.pi
    return -1 * diff

def y_angle(rvec):
    rotation_matrix, _ = cv2.Rodrigues(rvec)
    def rotationMatrixToEulerAngles(R) :
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        singular = sy < 1e-6
        if not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        return np.array([x, y, z])

    euler = rotationMatrixToEulerAngles(rotation_matrix)
    euler_degrees = [180*item/3.14159 for item in euler]
    return euler_degrees[2]

"""
Get width/length of robot room from markers
"""
def get_width_length(tvec_list):
    #WIDTH using euclidian distance
    a = np.linalg.norm(tvec_list[3] - tvec_list[2])
    b = np.linalg.norm(tvec_list[0] - tvec_list[1])
    width = (a+b)/2
    
    #LENGTH using euclidian distance
    c = np.linalg.norm(tvec_list[0] - tvec_list[2])
    d = np.linalg.norm(tvec_list[1] - tvec_list[3])
    length = (c+d)/2
    return width, length, a, b, c, d


"""
Rearranges vectors to be indexed by marker

"""
def clean_tvec_rvec(tvecs, rvecs):
    tvec_list = [0]*5
    for index, tvec in tvecs:
        tvec_list[index] = tvec
    rvec_list = [0]*5
    for index, rvec in rvecs:
        rvec_list[index] = rvec
    return tvec_list, rvec_list

"""
Calculates possible locations using intersection of 2 circles

"""
def calculate_possible_points(tvec_list, width, length):
    #for every corner, find the intersection
    corners =[(0, 0), (width, 0), (0, length), (width, length)]
    possible_points = []
    point_tvec = tvec_list[4]
    for i in range(4):
        for j in range(i+1, 4):
            x0, y0 = corners[i]
            r0 = np.linalg.norm(point_tvec-tvec_list[i])
            x1, y1 = corners[j]
            r1 = np.linalg.norm(point_tvec-tvec_list[j])
            returned = get_intersections(x0, y0, r0, x1, y1, r1)
            if returned != None:
                point1, point2 = returned
                possible_points.append(point1)
                possible_points.append(point2)
    return possible_points

"""
Cleans possible points computed with tvecs and returns estimated spot in row major order
tldr; vote on most likely location after intersection of 2 circles
INPUTS
    width - 23, 10 average
    length - 02, 13 average
    box_size - an integer, box robot lives in will be size x size, should be square...
    possible_points - possible points computed from intersection mess

"""

def clean_points_and_vote(width, length, box_size, possible_points):
    block_width = width/box_size
    block_length = length/box_size
    possible_coords = [0]*box_size*box_size
    max_votes = 0
    max_vote_index = -1;
    for x, y in possible_points:
        if x < 0 or y < 0:
            continue 
        x = math.floor(x/block_width)
        y = math.floor(y/block_length)
        if x > box_size-1 or y > box_size-1:
            continue
        # debug print
        # print(str(x) + ' ' + str(y))
        possible_coords[ y * box_size + x] += 1
        if possible_coords[ y * box_size + x] > max_votes:
            max_votes = possible_coords[ y * box_size + x]
            max_vote_index =  y * box_size + x
    x_avg = []
    y_avg = []
    
    for x, y in possible_points:
        if x < 0 or y < 0:
            continue 
        roundx = math.floor(x/block_width)
        roundy = math.floor(y/block_length)
        if roundx > box_size-1 or roundy > box_size-1:
            continue
#         if roundy * box_size + roundx == max_vote_index:
#             x_avg.append(x)
#             y_avg.append(y)
        x_avg.append(x)
        y_avg.append(y)
    return max_vote_index, sum(x_avg)/len(x_avg), sum(y_avg)/len(y_avg)



def clean_points_and_vote1(width, length, box_size, possible_points):
    block_width = width/box_size
    block_length = length/box_size
    possible_coords = [0]*box_size*box_size
    max_votes = 0
    max_vote_index = -1;
    #x = math.floor(x/block_width)
    #y = math.floor(y/block_length)
    definite_points = []
    sus_points = []
    for pair in possible_points:
        point1, point2 = pair
        if point1[0] < 0 or point1[1] < 0:
            definite_points.append(point2)
            continue
        if point2[0] < 0 or point2[1] < 0:
            definite_points.append(point1)
            continue
        sus_points.append(pair)
    print(definite_points)
    for pair in sus_points:
        print(pair)
    print('start filter')
    if len(definite_points) > 0:
        # filter by distance
        xavg = 0
        yavg = 0
        for point in definite_points:
            xavg += point[0]
            yavg += point[1]
        xavg = xavg/len(definite_points)
        yavg = yavg/len(definite_points)
        avgpt = [xavg, yavg]
        
        for pair in sus_points:
            dist0 = utils.find_distance(pair[0], avgpt)
            dist1 = utils.find_distance(pair[1], avgpt)
            if dist0 < dist1:
                definite_points.append(pair[0])
            else:
                definite_points.append(pair[1])
        
        #final average
        xavg = 0
        yavg = 0
        for point in definite_points:
            xavg += point[0]
            yavg += point[1]
        xavg = xavg/len(definite_points)
        yavg = yavg/len(definite_points)
        print(definite_points)
        return 0, xavg, yavg
    else:
         # otherwise just take all of them 
        print('fuck')
        return 0, 0 ,0

def calculate_possible_points1(tvec_list, width, length):
    #for every corner, find the intersection
    corners =[(0, 0), (width, 0), (0, length), (width, length)]
    possible_points = []
    point_tvec = tvec_list[4]
    for i in range(4):
        for j in range(i+1, 4):
            x0, y0 = corners[i]
            r0 = np.linalg.norm(point_tvec-tvec_list[i])
            x1, y1 = corners[j]
            r1 = np.linalg.norm(point_tvec-tvec_list[j])
            returned = utils.get_intersections(x0, y0, r0, x1, y1, r1)
            if returned != None:
                point1, point2 = returned
                possible_points.append([point1, point2])
    return possible_points

def pose_estimation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):

    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    return:-
    frame - The frame with the axis drawn on it
    '''

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()


    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)
    
    rvecs = []
    tvecs = []
        # If markers are detected
    if len(corners) > 0:
        #print(str(ids))
        for i in range(0, len(ids)):
            
            
            # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients,
                                                                       distortion_coefficients)
            # Draw a square around the markers
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            # Draw Axis
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)
            
            # draw marker number
            (topLeft, topRight, bottomRight, bottomLeft) = corners[i][0]
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cv2.putText(frame, str(ids[i]),
            (topLeft[0] - 15, topLeft[1]), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)
            
            entry = [ids[i][0], rvec[0][0]]
            rvecs.append(entry)
            entry = [ids[i][0], tvec[0][0]]
            tvecs.append(entry)
    return frame, rvecs, tvecs

def calculate_possible_points1(tvec_list, width, length):
    #for every corner, find the intersection
    corners =[(0, 0), (width, 0), (0, length), (width, length)]
    possible_points = []
    point_tvec = tvec_list[4]
    for i in range(4):
        for j in range(i+1, 4):
            x0, y0 = corners[i]
            r0 = np.linalg.norm(point_tvec-tvec_list[i])
            x1, y1 = corners[j]
            r1 = np.linalg.norm(point_tvec-tvec_list[j])
            returned = utils.get_intersections(x0, y0, r0, x1, y1, r1)
            if returned != None:
                point1, point2 = returned
                possible_points.append([point1, point2])
    return possible_points

def clean_points_and_vote1(width, length, box_size, possible_points):
    block_width = width/box_size
    block_length = length/box_size
    possible_coords = [0]*box_size*box_size
    max_votes = 0
    max_vote_index = -1;
    #x = math.floor(x/block_width)
    #y = math.floor(y/block_length)
    definite_points = []
    sus_points = []
    for pair in possible_points:
        point1, point2 = pair
        if point1[0] < 0 or point1[1] < 0:
            definite_points.append(point2)
            continue
        if point2[0] < 0 or point2[1] < 0:
            definite_points.append(point1)
            continue
        sus_points.append(pair)
    print(definite_points)
    for pair in sus_points:
        print(pair)
    print('start filter')
    if len(definite_points) > 0:
        # filter by distance
        xavg = 0
        yavg = 0
        for point in definite_points:
            xavg += point[0]
            yavg += point[1]
        xavg = xavg/len(definite_points)
        yavg = yavg/len(definite_points)
        avgpt = [xavg, yavg]
        
        for pair in sus_points:
            dist0 = utils.find_distance(pair[0], avgpt)
            dist1 = utils.find_distance(pair[1], avgpt)
            if dist0 < dist1:
                definite_points.append(pair[0])
            else:
                definite_points.append(pair[1])
        
        
        #final average
        xavg = 0
        yavg = 0
        for point in definite_points:
            xavg += point[0]
            yavg += point[1]
        xavg = xavg/len(definite_points)
        yavg = yavg/len(definite_points)
        print(definite_points)
        return 0, xavg, yavg
    else:
         # otherwise just take all of them 
        print('fuck')
        return 0, 0 ,0
    
def find_location(tags, distances):
    A = []
    b = []
    for i in range(1, len(tags)):
        #print(i)
        A.append([2 * (tags[0][0] - tags[i][0]), 2 * (tags[0][1] - tags[i][1])])
        b.append(np.linalg.norm(tags[0]) ** 2 - np.linalg.norm(tags[i]) ** 2 - distances[0] ** 2 + distances[i] ** 2)
    return np.linalg.lstsq(A, b)[0]
    
def calculate_location(tvec_list, width, length):
    #for every corner, find the intersection
    point_tvec = tvec_list[4]
    corners =[(0, length), (width, length), (0, 0), (width, 0)]
    distances = []
    for i in range(len(corners)):
        #print(i)
        distance = np.linalg.norm(point_tvec-tvec_list[i])
        #print(distance)
        distances.append(distance)
    return find_location(corners, distances)