# Python code for Multiple Color Detection
import numpy as np
import cv2
from math import *
import urllib
import urllib.request
import json
import time
import serial
import pandas as pd
from itertools import product
import os
os.system('cls' if os.name == 'nt' else 'clear')



# Trajectory Properties:
red_rx = 30
red_ry = 55
blue_rx = 30
blue_ry = 50
orange_rx = 30
orange_ry = 40

manualControl = False
df = pd.DataFrame([], columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
robot_data_1 = []
robot_data_2 = []
robot_data_3 = []
beginTime = time.time()
last_step_time = time.time()
center_x = 155
center_y = 120
Line_x = 35
Line_y = 145
circle_radius_x = 45
circle_radius_y = 30
x_Center1 = 0
x_Center2 = 0
x_Center3 = 0
y_Center1 = 0
y_Center2 = 0
y_Center3 = 0
green_angle = 0
yellow_angle = 0
pink_angle = 0
# --------- Kinematic Variables
t1 = 0
derivationFlag_1 = 0
derivationFlag_2 = 0
derivationFlag_3 = 0
ls1 = 0.55
ls2 = 1.35
ls31 = 2.75
ls32 = 1.65
le1 = 0.35
le2 = 0.35
le31 = 1.5
le32 = 1.2

# k1 = 50.0
# k2 = 50.0
# k3 = 2.0

k1_1 = 40.0
k2_1 = 40.0
k3_1 = 2.0

k1_2 = 50.0
k2_2 = 50.0
k3_2 = 2.0

k1_3 = 50.0
k2_3 = 50.0
k3_3 = 2.0

k4 = 1.0
k5 = 1.0
k41 = 2
k42 = 2
width = 0.125
r = 0.06
A = 0
B = 0
# speed = 0.1
speed = pi/30
M_PI = pi
M_2PI = 2*pi


x_1 = 0.35
y_1 = 0
angle_1 = 3.14/2
Xi1_1 = 0
Xi2_1 = 0
lastalpha_1=0
alpha_1 = 0
z1_1 = 0
z2_1 = 0
alphadot_1=0
V_1 = 0
w_1 = 0
_xcd_1 = 0
_ycd_1 = 0
_thetad_1 = 0
_wd_1 = 0
_x2d_1 = 0
_x3d_1 = 0
_x2_1 = 0
_x3_1 = 0

x_2 = 0.35
y_2 = 0
angle_2 = 3.14/2
Xi1_2 = 0
Xi2_2 = 0
lastalpha_2=0
alpha_2 = 0
z1_2 = 0
z2_2 = 0
alphadot_2=0
V_2 = 0
w_2 = 0
_xcd_2 = 0
_ycd_2 = 0
_thetad_2 = 0
_wd_2 = 0
_x2d_2 = 0
_x3d_2 = 0
_x2_2 = 0
_x3_2 = 0

x_3 = 0.35
y_3 = 0
angle_3 = 3.14/2
Xi1_3 = 0
Xi2_3 = 0
lastalpha_3=0
alpha_3 = 0
z1_3 = 0
z2_3 = 0
alphadot_3=0
V_3 = 0
w_3 = 0
_xcd_3 = 0
_ycd_3 = 0
_thetad_3 = 0
_wd_3 = 0
_x2d_3 = 0
_x3d_3 = 0
_x2_3 = 0
_x3_3 = 0


# --------- Math Additional Functions
def constrainAngle(x_1):
    x_1 = fmod(x_1 + M_PI,M_2PI)
    if (x_1 < 0):
        x_1 += M_2PI
    return x_1 - M_PI
def angleConv(angle_1):
    return fmod(constrainAngle(angle_1),M_2PI)
def angleDiff(a,b):
    dif = fmod(b - a + M_PI,M_2PI)
    if (dif < 0):
        dif += M_2PI
    return dif - M_PI
def unwrap(previousAngle,newAngle):
    return previousAngle - angleDiff(newAngle,angleConv(previousAngle))
def sin2(val):
	return pow(sin(val), 2)
def cos2(val):
	return pow(cos(val), 2)
def absf(input):
	if(input >= 0): return input
	return -input

# --------- Kinematic Functions
# Robot 1 = Red
def xcd_1():
    return red_rx/100 * cos(t1*speed)
def ycd_1():
	return red_ry/100 * sin(t1*speed)
def thetad_1():
	theta = atan2(red_ry/100*cos(t1*speed), -red_rx/100*sin(t1*speed))
	# theta = atan2(0.45*cos(t1*speed), -0.30*sin(t1*speed))
	return unwrap(_thetad_1, theta)
def wd_1():
    a = red_ry/100
    b = red_rx/100
    v = speed
    return (a*b*v*(sin2(v*t1)+cos2(v*t1)))/(b**2*sin2(v*t1)+a**2*cos2(v*t1))
	# return (6*speed * (sin2(t1*speed) +   cos2(t1*speed))) /	(4*sin2(t1*speed) + 9*cos2(t1*speed))
def x2d_1():
	return (_xcd_1 * cos(_thetad_1)) + (_ycd_1 * sin(_thetad_1))
def x3d_1():
	return (_xcd_1 * sin(_thetad_1)) - (_ycd_1 * cos(_thetad_1))
def x2_1(xc, yc):
	return (xc * cos(angle_1)) + (yc * sin(angle_1))
def x3_1(xc, yc):
	return (xc * sin(angle_1)) - (yc * cos(angle_1))

# Robot 2 = Blue
def xcd_2():
	return blue_rx/100 * cos(t1*speed)
def ycd_2():
	return blue_ry/100 * sin(t1*speed)
def thetad_2():
	theta = atan2(blue_ry/100*cos(t1*speed), -blue_rx/100*sin(t1*speed))
	return unwrap(_thetad_2, theta)
def wd_2():
    a = blue_ry/100
    b = blue_rx/100
    v = speed
    return (a*b*v*(sin2(v*t1)+cos2(v*t1)))/(b**2*sin2(v*t1)+a**2*cos2(v*t1))
	# return (6*speed * (sin2(t1*speed) +   cos2(t1*speed))) /	(4*sin2(t1*speed) + 9*cos2(t1*speed))
def x2d_2():
	return (_xcd_2 * cos(_thetad_2)) + (_ycd_2 * sin(_thetad_2))
def x3d_2():
	return (_xcd_2 * sin(_thetad_2)) - (_ycd_2 * cos(_thetad_2))
def x2_2(xc, yc):
	return (xc * cos(angle_2)) + (yc * sin(angle_2))
def x3_2(xc, yc):
	return (xc * sin(angle_2)) - (yc * cos(angle_2))

# Robot 3 = Orange
def xcd_3():
	return orange_rx/100 * cos(t1*speed)
def ycd_3():
	return orange_ry/100 * sin(t1*speed)
def thetad_3():
	theta = atan2(orange_ry/100*cos(t1*speed), -orange_rx/100*sin(t1*speed))
	return unwrap(_thetad_3, theta)
def wd_3():
    a = orange_ry/100
    b = orange_rx/100
    v = speed
    return (a*b*v*(sin2(v*t1)+cos2(v*t1)))/(b**2*sin2(v*t1)+a**2*cos2(v*t1))
	# return (6*speed * (sin2(t1*speed) +   cos2(t1*speed))) /	(4*sin2(t1*speed) + 9*cos2(t1*speed))
def x2d_3():
	return (_xcd_3 * cos(_thetad_3)) + (_ycd_3 * sin(_thetad_3))
def x3d_3():
	return (_xcd_3 * sin(_thetad_3)) - (_ycd_3 * cos(_thetad_3))
def x2_3(xc, yc):
	return (xc * cos(angle_3)) + (yc * sin(angle_3))
def x3_3(xc, yc):
	return (xc * sin(angle_3)) - (yc * cos(angle_3))




with open("data.json", "r") as openfile:
    # Reading from json file
    hsvColors = json.load(openfile)
# Capturing video through webcam
webcam = cv2.VideoCapture(0)

# ser = serial.Serial()
# ser.baudrate = 115200
# ser.port = 'COM11'
ser = [
    serial.Serial(timeout=0.1), 
    serial.Serial(timeout=0.1), 
    serial.Serial(timeout=0.1),
]
ser[0].baudrate = 115200
ser[1].baudrate = 115200
ser[2].baudrate = 115200
ser[0].port = 'COM20'
ser[1].port = 'COM13'
ser[2].port = 'COM18'
robot_id = 2

try:
    ser[0].open()
except:
    print(f'Robot 1 is not connected !!')
try:
    ser[1].open()
except:
    print(f'Robot 2 is not connected !!')
try:
    ser[2].open()
except:
    print(f'Robot 3 is not connected !!')
url = "http://192.168.18.77:8080/shot.jpg"

packet_green = [0, 0, 0, 0]
x_Center1, x_orange_center, y_orange_center, x_red_center, x_blue_center, y_red_center = (
    0,
    0,
    0,
    0,
    0,
    0,
)

y_blue_center = 0

x_centerPoint = 543
y_centerPoint = 328

red_color = (0, 0, 255)
blue_color = (255, 0, 0)
orange_color = (0, 188, 255)


RPM_Left_1 = 0
RPM_Right_1 = 0
RPM_Left_2 = 0
RPM_Right_2 = 0
RPM_Left_3 = 0
RPM_Right_3 = 0

def colorSetHSV(color, upperOrLower):
    with open("data.json", "r") as openfile:
        # Reading from json file
        hsvColors = json.load(openfile)
    output = np.array(
        [
            hsvColors[color][upperOrLower]["h"],
            hsvColors[color][upperOrLower]["s"],
            hsvColors[color][upperOrLower]["v"],
        ],
        np.uint8,
    )
    return output
def Send_RPM_to_Robot(RPM_L, RPM_R, id):
    global packet_green
    if RPM_L > 90: RPM_L = 90
    if RPM_L <-90: RPM_L = -90
    if RPM_R > 90: RPM_R = 90
    if RPM_R <-90: RPM_R = -90
    RPM_L  = round(RPM_L, 2) * 100 + 9000
    RPM_R = round(RPM_R, 2 ) * 100 + 9000
    packet_green[0] = int(RPM_L).to_bytes(2, "little",signed=True)[0]
    packet_green[1] = int(RPM_L).to_bytes(2, "little",signed=True)[1]

    packet_green[2] = int(RPM_R).to_bytes(2, "little",signed=True)[0]
    packet_green[3] = int(RPM_R).to_bytes(2, "little",signed=True)[1]

    try:
        ser[id].write(packet_green)
    except:
        pass
        # print(f'Robot {id+1} is not connected !!')

# Start a while loop
while 1:
    start = time.time()
    ret, imageFrame = webcam.read()#cv2.imdecode(imgNp, -1)
    hsvFrame    = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_mask    = cv2.inRange(hsvFrame, colorSetHSV("red", "lower"), colorSetHSV("red", "upper"))
    green_mask  = cv2.inRange(hsvFrame, colorSetHSV("green", "lower"), colorSetHSV("green", "upper"))
    blue_mask   = cv2.inRange(hsvFrame, colorSetHSV("blue", "lower"), colorSetHSV("blue", "upper"))
    yellow_mask = cv2.inRange(hsvFrame, colorSetHSV("yellow", "lower"), colorSetHSV("yellow", "upper"))
    orange_mask = cv2.inRange(hsvFrame, colorSetHSV("orange", "lower"), colorSetHSV("orange", "upper"))
    pink_mask   = cv2.inRange(hsvFrame, colorSetHSV("pink", "lower"), colorSetHSV("pink", "upper"))
    kernal      = np.ones((5, 5), "uint8")
    red_mask    = cv2.dilate(red_mask, kernal)
    res_red     = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)
    green_mask  = cv2.dilate(green_mask, kernal)
    green_mask  = cv2.erode(green_mask, kernal)
    res_green   = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)
    blue_mask   = cv2.dilate(blue_mask, kernal)
    blue_mask   = cv2.erode(blue_mask, kernal)
    res_blue    = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    # -------------- red
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_red, y_red, w_red, h_red = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 0, 255), 2)
            
            x_red_center = x_red + (w_red / 2)
            y_red_center = y_red + (h_red / 2)

            cv2.putText(
                imageFrame,
                "1",
                (x_red, y_red),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
            )

    # -------------- green
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_green, y_green, w_green, h_green = cv2.boundingRect(contour)
            if w_green > 200 or h_green > 200:
                break
            cv2.drawContours(imageFrame, [box], 0, (0, 255, 0), 2)
            
            x_green_center = (x_green + (w_green / 2))
            y_green_center = (y_green + (h_green / 2))
                  
            x_Center1 = round(x_red_center * 0.489 , 2)
            y_Center1 = round((480 - y_red_center) * 0.489 , 2)            
            
            green_angle = degrees(atan2((y_green_center - y_red_center), (x_green_center - x_red_center)))
            if green_angle < 0:
                green_angle = green_angle + 360
            green_angle = 360 - green_angle
            # cv2.putText(
            #     imageFrame,
            #     # # "2",
            #     str(x_Center1)
            #     + " "
            #     + str(y_Center1)
            #     + " "
            #     + str(int(green_angle))
            #     + " "
            #     + str(round(time.time() - beginTime, 1)),
            #     (50, 50),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1.0,
            #     (200, 255, 200),
            # )

    # -------------- blue
    contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_blue, y_blue, w_blue, h_blue = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (255, 0, 0), 2)

            x_blue_center = x_blue + (w_blue / 2)
            y_blue_center = y_blue + (h_blue / 2)

            cv2.putText(
                imageFrame,
                "2",
                (x_blue, y_blue),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 0, 0),
            )

    # -------------- yellow
    contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_yellow, y_yellow, w_yellow, h_yellow = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 255, 255), 2)
            

            x_yellow_center = x_yellow + (w_yellow / 2)
            y_yellow_center = y_yellow + (h_yellow / 2)

            x_Center2 = round(x_blue_center * 0.489 , 2)
            y_Center2 = round((480 - y_blue_center) * 0.489 , 2)  

            yellow_angle = degrees(atan2((y_yellow_center - y_blue_center), (x_yellow_center - x_blue_center)))
            if yellow_angle < 0:
                yellow_angle = yellow_angle + 360
            yellow_angle = 360 - yellow_angle
            
            # cv2.putText(
            #     imageFrame,
            #     "yellow " + " " + str(yellow_angle),
            #     (x_yellow, y_yellow),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1.0,
            #     (0, 255, 255),
            # )

    # -------------- orange
    contours, hierarchy = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_orange, y_orange, w_orange, h_orange = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 188, 255), 2)
            
            x_orange_center = x_orange + (w_orange / 2)
            y_orange_center = y_orange + (h_orange / 2)

            cv2.putText(
                imageFrame,
                "3",
                (x_orange, y_orange),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 188, 255),
            )

    # -------------- pink
    contours, hierarchy = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_pink, y_pink, w_pink, h_pink = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (127, 157, 255), 2)
            
            x_pink_center = x_pink + (w_pink / 2)
            y_pink_center = y_pink + (h_pink / 2)
            
            x_Center3 = round(x_orange_center * 0.489 , 2)
            y_Center3 = round((480 - y_orange_center) * 0.489 , 2)  

            pink_angle = degrees(atan2((y_pink_center - y_orange_center), (x_pink_center - x_orange_center)))
            if pink_angle < 0:
                pink_angle = pink_angle + 360
            pink_angle = 360 - pink_angle
            # cv2.putText(
            #     imageFrame,
            #     "pink " + " " + str(pink_angle),
            #     (x_pink, y_pink),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1.0,
            #     (127, 157, 255),
            # )
            cv2.putText(
                imageFrame,
                # str(x_Center3)
                # + " "
                # + str(y_Center3)
                # + " "
                # + str(int(pink_angle))
                "time: "
                + str(round(time.time() - beginTime, 1)),
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (200, 255, 200),
            )


    # -------------- Controller

    t1 = time.time() - beginTime
    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-------------- Robot 1
    x_1 = (x_Center1 - center_x - 90)/100
    y_1 = (y_Center1 - center_y)/100
    angle_1 = unwrap(angle_1, radians(green_angle))

    _xcd_1 = xcd_1()
    _ycd_1 = ycd_1()
    _thetad_1 = thetad_1()
    _wd_1 = wd_1()
    _x2d_1 = x2d_1()
    _x3d_1 = x3d_1()
    _x2_1 = x2_1(x_1, y_1)
    _x3_1 = x3_1(x_1, y_1)

    Xi1_1 = _wd_1 + (k3_1 * (_thetad_1 - angle_1))
    z1_1 = _x3d_1 - _x3_1
    A = pow(le1,2) - pow(z1_1,2)
    alpha_1 = _x2d_1 + (A * k1_1 * z1_1 * _wd_1)
    z2_1 = alpha_1 - _x2_1
    B = pow(le2,2) - pow(z2_1,2)

    if(derivationFlag_1):
        alphadot_1 = (alpha_1 - lastalpha_1)/0.1
        lastalpha_1 = alpha_1
        derivationFlag_1 = 0

    Xi2_1 = alphadot_1 + (B * k2_1 * z2_1 * pow(_wd_1,2)) + ((B / A) * k5 * z1_1 * _wd_1)
    
    V_1 = _x3_1 * Xi1_1 + k4 * Xi2_1
    w_1 = Xi1_1

    RPM_Right_1 = ((V_1 + (width*w_1))/r) * 9.55
    RPM_Left_1 = ((V_1 - (width*w_1))/r) * 9.55

    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-------------- Robot 2
    x_2 = (x_Center2 - center_x)/100
    y_2 = (y_Center2 - center_y)/100
    angle_2 = unwrap(angle_2, radians(yellow_angle))

    _xcd_2 = xcd_2()
    _ycd_2 = ycd_2()
    _thetad_2 = thetad_2()
    _wd_2 = wd_2()
    _x2d_2 = x2d_2()
    _x3d_2 = x3d_2()
    _x2_2 = x2_2(x_2, y_2)
    _x3_2 = x3_2(x_2, y_2)

    Xi1_2 = _wd_2 + (k3_2 * (_thetad_2 - angle_2))
    z1_2 = _x3d_2 - _x3_2
    A = pow(le1,2) - pow(z1_2,2)
    alpha_2 = _x2d_2 + (A * k1_2 * z1_2 * _wd_2)
    z2_2 = alpha_2 - _x2_2
    B = pow(le2,2) - pow(z2_2,2)

    if(derivationFlag_2):
        alphadot_2 = (alpha_2 - lastalpha_2)/0.1
        lastalpha_2 = alpha_2
        derivationFlag_2 = 0

    Xi2_2 = alphadot_2 + (B * k2_2 * z2_2 * pow(_wd_2,2)) + ((B / A) * k5 * z1_2 * _wd_2)
    
    V_2 = _x3_2 * Xi1_2 + k4 * Xi2_2
    w_2 = Xi1_2

    RPM_Right_2 = ((V_2 + (width*w_2))/r) * 9.55
    RPM_Left_2 = ((V_2 - (width*w_2))/r) * 9.55

    # \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\-------------- Robot 3
    x_3 = (x_Center3 - center_x + 90)/100
    y_3 = (y_Center3 - center_y)/100
    angle_3 = unwrap(angle_3, radians(pink_angle))

    _xcd_3 = xcd_3()
    _ycd_3 = ycd_3()
    _thetad_3 = thetad_3()
    _wd_3 = wd_3()
    _x2d_3 = x2d_3()
    _x3d_3 = x3d_3()
    _x2_3 = x2_3(x_3, y_3)
    _x3_3 = x3_3(x_3, y_3)

    Xi1_3 = _wd_3 + (k3_3 * (_thetad_3 - angle_3))
    z1_3 = _x3d_3 - _x3_3
    A = pow(le1,2) - pow(z1_3,2)
    alpha_3 = _x2d_3 + (A * k1_3 * z1_3 * _wd_3)
    z2_3 = alpha_3 - _x2_3
    B = pow(le2,2) - pow(z2_3,2)

    if(derivationFlag_3):
        alphadot_3 = (alpha_3 - lastalpha_3)/0.1
        lastalpha_3 = alpha_3
        derivationFlag_3 = 0

    Xi2_3 = alphadot_3 + (B * k2_3 * z2_3 * pow(_wd_3,2)) + ((B / A) * k5 * z1_3 * _wd_3)
    
    V_3 = _x3_3 * Xi1_3 + k4 * Xi2_3
    w_3 = Xi1_3

    RPM_Right_3 = ((V_3 + (width*w_3))/r) * 9.55
    RPM_Left_3 = ((V_3 - (width*w_3))/r) * 9.55

    # print(f"V_1:{round(V_1,2)} \tw:{round(w_1,2)} \tRPM_R:{round(RPM_Right_1,2)} \tRPM_L:{round(RPM_Left_1,2)} \t")
    # print(f"V_1:{round(V_1,2)} \tw:{round(w_1,2)} \t angle_1:{round(angle_1,2)} \tthetaD:{round(_thetad_1,2)}")
    # print(f"x_1:{round(x_1,2)} \y_1:{round(y_1,2)} \t angle_1:{round(angle_1,2)}")
    # print(f"x_2:{round(x_2,2)} \y_2:{round(y_2,2)} \t angle_2:{round(angle_2,2)}")

    # -------------- Sending Data To MCU
    if not manualControl:
        Send_RPM_to_Robot(RPM_Left_1, RPM_Right_1, 0)
        Send_RPM_to_Robot(RPM_Left_2, RPM_Right_2, 1)
        Send_RPM_to_Robot(RPM_Left_3, RPM_Right_3, 2)

    # -------------- Record Robot Data
    if time.time() - last_step_time > 0.1:
        derivationFlag_1 = 1
        derivationFlag_2 = 1
        derivationFlag_3 = 1
        Xd = _xcd_1*100 + center_x
        Yd = _ycd_1*100 + center_y
        thetaD = _thetad_1
        
        if thetaD < 0: thetaD += 2*pi
        robot_data_1.append([
            t1, 
            x_1*100 + center_x, 
            y_1*100 + center_y, 
            degrees(angle_1),
            Xd, 
            Yd,
            x_1*100 + center_x - Xd,
            y_1*100 + center_y - Yd,
            angle_1,
            thetaD,
            angle_1 - thetaD,
            _x2_1,
            _x3_1,
            _x2d_1,
            _x3d_1
        ])

        Xd = _xcd_2*100 + center_x
        Yd = _ycd_2*100 + center_y
        thetaD = _thetad_2
        
        if thetaD < 0: thetaD += 2*pi
        robot_data_2.append([
            t1, 
            x_2*100 + center_x, 
            y_2*100 + center_y, 
            degrees(angle_2),
            Xd, 
            Yd,
            x_2*100 + center_x - Xd,
            y_2*100 + center_y - Yd,
            angle_2,
            thetaD,
            angle_2 - thetaD,
            _x2_2,
            _x3_2,
            _x2d_2,
            _x3d_2
        ])

        Xd = _xcd_3*100 + center_x
        Yd = _ycd_3*100 + center_y
        thetaD = _thetad_3
        
        if thetaD < 0: thetaD += 2*pi
        robot_data_3.append([
            t1, 
            x_3*100 + center_x, 
            y_3*100 + center_y, 
            degrees(angle_3),
            Xd, 
            Yd,
            x_3*100 + center_x - Xd,
            y_3*100 + center_y - Yd,
            angle_3,
            thetaD,
            angle_3 - thetaD,
            _x2_3,
            _x3_3,
            _x2d_3,
            _x3d_3
        ])
        last_step_time = time.time()
    
    end = time.time()
    # print((end - start))


    # -------------- Event Handlers
    imageFrame = cv2.ellipse(imageFrame, (int((center_x-90)/0.489), int(480 - center_y/0.489)), (int(orange_rx/0.489), int(orange_ry/0.489)), 0, 0, 360, orange_color, 2) 
    imageFrame = cv2.ellipse(imageFrame, (int(center_x/0.489), int(480 - center_y/0.489)), (int(blue_rx/0.489), int(blue_ry/0.489)), 0, 0, 360, blue_color, 2) 
    imageFrame = cv2.ellipse(imageFrame, (int((center_x+90)/0.489), int(480 - center_y/0.489)), (int(red_rx/0.489), int(red_ry/0.489)), 0, 0, 360, red_color, 2) 
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    key = cv2.waitKey(1)
    # Program Termination
    if key & 0xff == 27:
        Send_RPM_to_Robot(0, 0, 0)
        Send_RPM_to_Robot(0, 0, 1)
        Send_RPM_to_Robot(0, 0, 2)
        break
    if key == ord('s') and not manualControl:
        df_1 = pd.DataFrame(robot_data_1, columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
        df_2 = pd.DataFrame(robot_data_2, columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
        df_3 = pd.DataFrame(robot_data_3, columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
        try:
            df_1.to_excel('./recordings/data_1.xlsx', sheet_name='Robot Positions')
            df_2.to_excel('./recordings/data_2.xlsx', sheet_name='Robot Positions')
            df_3.to_excel('./recordings/data_3.xlsx', sheet_name='Robot Positions')
            print('Excel Saved')
            Send_RPM_to_Robot(0, 0, 0)
            Send_RPM_to_Robot(0, 0, 1)
            Send_RPM_to_Robot(0, 0, 2)
            break
        except:
            print('Can not save file. Permission denied !!!!!!!')
    if key == ord('w') and manualControl:
        Send_RPM_to_Robot(40, 40, robot_id)
    if key == ord('s') and manualControl:
        Send_RPM_to_Robot(-40, -40, robot_id)
    if key == ord('a') and manualControl:
        Send_RPM_to_Robot(-40, 40, robot_id)
    if key == ord('d') and manualControl:
        Send_RPM_to_Robot(40, -40, robot_id)
    if key == ord(' ') and manualControl:
        Send_RPM_to_Robot(0, 0, robot_id)
    if key == ord('m'):
        manualControl = not manualControl
        Send_RPM_to_Robot(0, 0, 0)
        Send_RPM_to_Robot(0, 0, 1)
        Send_RPM_to_Robot(0, 0, 2)
    if key == ord('1'):
        robot_id = 0
    if key == ord('2'):
        robot_id = 1
    if key == ord('3'):
        robot_id = 2
    
#cap.release()
cv2.destroyAllWindows()
ser[0].close()
ser[1].close()
ser[2].close()