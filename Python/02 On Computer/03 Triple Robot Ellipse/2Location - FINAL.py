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

manualControl = False
df = pd.DataFrame([], columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
robot_data = []
beginTime = time.time()
last_step_time = time.time()
center_x = 155
center_y = 90
Line_x = 35
Line_y = 145
circle_radius_x = 45
circle_radius_y = 30
x_Center1 = 0
y_Center1 = 0
green_angle = 0

# --------- Kinematic Variables
t1 = 0
derivationFlag = 0
ls1 = 0.55
ls2 = 1.35
ls31 = 2.75
ls32 = 1.65
le1 = 0.35
le2 = 0.35
le31 = 1.5
le32 = 1.2
k1 = 50.0
k2 = 50.0
k3 = 2.0
k4 = 1.0
k5 = 1.0
k41 = 2
k42 = 2
width = 0.125
r = 0.06
A = 0
B = 0
speed = 0.1
M_PI = pi
M_2PI = 2*pi


x_1 = 0.55
x_2 = 0.55
x_3 = 0.55
y_1 = 0
y_2 = 0
y_3 = 0
angle_1 = 3.14/2
angle_2 = 3.14/2
angle_3 = 3.14/2
Xi1_1 = 0
Xi1_2 = 0
Xi1_3 = 0
Xi2_1 = 0
Xi2_2 = 0
Xi2_3 = 0
lastalpha_1 = 0
lastalpha_2 = 0
lastalpha_3 = 0
alpha_1 = 0
alpha_2 = 0
alpha_3 = 0
z1_1 = 0
z1_2 = 0
z1_3 = 0
z2_1 = 0
z2_2 = 0
z2_3 = 0
alphadot_1 = 0
alphadot_2 = 0
alphadot_3 = 0
V_1 = 0
V_2 = 0
V_3 = 0
w_1 = 0
w_2 = 0
w_3 = 0
_xcd_1 = 0
_xcd_2 = 0
_xcd_3 = 0
_ycd_1 = 0
_ycd_2 = 0
_ycd_3 = 0
_thetad_1 = 0
_thetad_2 = 0
_thetad_3 = 0
_wd_1 = 0
_wd_2 = 0
_wd_3 = 0
_x2d_1 = 0
_x2d_2 = 0
_x2d_3 = 0
_x3d_1 = 0
_x3d_2 = 0
_x3d_3 = 0
_x2_1 = 0
_x2_2 = 0
_x2_3 = 0
_x3_1 = 0
_x3_2 = 0
_x3_3 = 0


# --------- Math Additional Functions
def constrainAngle(x):
    x = fmod(x + M_PI,M_2PI)
    if (x < 0):
        x += M_2PI
    return x - M_PI
def angleConv(angle):
    return fmod(constrainAngle(angle),M_2PI)
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
def xcd1():
	return 0.45 * cos(t1*speed)
def ycd1():
	return 0.30 * sin(t1*speed)
def xcd2():
	return 0.45 * cos(t1*speed)
def ycd2():
	return 0.30 * sin(t1*speed)
def xcd3():
	return 0.45 * cos(t1*speed)
def ycd3():
	return 0.30 * sin(t1*speed)
def thetad():
	theta = atan2(0.30*cos(t1*speed), -0.45*sin(t1*speed))
	return unwrap(_thetad_1, theta)
def wd():
	return (6*speed * (sin2(t1*speed) +   cos2(t1*speed))) /	(9*sin2(t1*speed) + 4*cos2(t1*speed))
def x2d():
	return (_xcd_1 * cos(_thetad_1)) + (_ycd_1 * sin(_thetad_1))
def x3d():
	return (_xcd_1 * sin(_thetad_1)) - (_ycd_1 * cos(_thetad_1))
def x2(xc, yc):
	return (xc * cos(angle_1)) + (yc * sin(angle_1))
def x3(xc, yc):
	return (xc * sin(angle_1)) - (yc * cos(angle_1))




with open("data.json", "r") as openfile:
    # Reading from json file
    hsvColors = json.load(openfile)
# Capturing video through webcam
webcam = cv2.VideoCapture(0)

# ser = serial.Serial()
# ser.baudrate = 115200
# ser.port = 'COM11'
ser = [
    serial.Serial(timeout=0.5), 
    serial.Serial(timeout=0.5), 
    serial.Serial(timeout=0.5),
]
ser[0].baudrate = 115200
ser[1].baudrate = 115200
ser[2].baudrate = 115200
ser[0].port = 'COM20'
ser[1].port = 'COM18'
ser[2].port = 'COM29'
robot_id = 0

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

RPM_Left = 0
RPM_Right = 0

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
def Send_RPM_to_Robot(rpmLeft, rpmRight, id):
    global packet_green
    if rpmLeft > 90: rpmLeft = 90
    if rpmLeft <-90: rpmLeft = -90
    if rpmRight > 90: rpmRight = 90
    if rpmRight <-90: rpmRight = -90
    rpmLeft  = round(rpmLeft, 2) * 100 + 9000
    rpmRight = round(rpmRight, 2 ) * 100 + 9000
    packet_green[0] = int(rpmLeft).to_bytes(2, "little",signed=True)[0]
    packet_green[1] = int(rpmLeft).to_bytes(2, "little",signed=True)[1]

    packet_green[2] = int(rpmRight).to_bytes(2, "little",signed=True)[0]
    packet_green[3] = int(rpmRight).to_bytes(2, "little",signed=True)[1]

    try:
        ser[id].open()
        ser[id].write(packet_green)
        ser[id].close()
    except:
        print(f'Robot {id+1} is not connected !!')


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
        if area > 50 and area<50000:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_red, y_red, w_red, h_red = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 0, 255), 2)
            
            x_red_center = x_red + (w_red / 2)
            y_red_center = y_red + (h_red / 2)

            cv2.putText(
                imageFrame,
                "Red ",
                (x_red, y_red),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
            )

    # -------------- green
    contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 50 and area<50000:
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
            cv2.putText(
                imageFrame,
                "Green "
                + str(x_Center1)
                + " "
                + str(y_Center1)
                + " "
                + str(int(green_angle))
                + " "
                + str(round(time.time() - beginTime, 1)),
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
            )

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
                "Blue ",
                (x_blue, y_blue),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (255, 0, 0),
            )

    # -------------- yellow
    contours, hierarchy = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 600 and area<900:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_yellow, y_yellow, w_yellow, h_yellow = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 255, 255), 2)
            # red_angle = int(rect[2])
            # getOrientation(contour, imageFrame)

            x_yellow_center = x_yellow + (w_yellow / 2)
            y_yellow_center = y_yellow + (h_yellow / 2)

            x_Center2 = (x_yellow_center + x_blue_center) / 2
            y_Center2 = (y_yellow_center + y_blue_center) / 2

            yellow_angle = degrees(
                atan2(
                    (y_yellow_center - y_blue_center), (x_yellow_center - x_blue_center)
                )
            )
            if yellow_angle < 0:
                yellow_angle = yellow_angle + 360

            cv2.putText(
                imageFrame,
                "yellow " + " " + str(yellow_angle),
                (x_yellow, y_yellow),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
            )

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
                "orange ",
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

            pink_angle = degrees(
                atan2(
                    (y_orange_center - y_pink_center), (x_orange_center - x_pink_center)
                )
            )
            if pink_angle < 0:
                pink_angle = pink_angle + 360

            cv2.putText(
                imageFrame,
                "pink " + " " + str(pink_angle),
                (x_pink, y_pink),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (127, 157, 255),
            )


    # -------------- Controller
    x_1 = (x_Center1 - center_x)/100
    y_1 = (y_Center1 - center_y)/100
    angle_1 = unwrap(angle_1, radians(green_angle))
    t1 = time.time() - beginTime

    _xcd_1 = xcd1()
    _ycd_1 = ycd1()
    _thetad_1 = thetad()
    _wd_1 = wd()
    _x2d_1 = x2d()
    _x3d_1 = x3d()
    _x2_1 = x2(x_1, y_1)
    _x3_1 = x3(x_1, y_1)

    Xi1_1 = _wd_1 + (k3 * (_thetad_1 - angle_1))
    z1_1 = _x3d_1 - _x3_1
    A = pow(le1,2) - pow(z1_1,2)
    alpha = _x2d_1 + (A * k1 * z1_1 * _wd_1)
    z2_1 = alpha_1 - _x2_1
    B = pow(le2,2) - pow(z2_1,2)

    if(derivationFlag):
        alphadot_1 = (alpha_1 - lastalpha_1)/0.1
        lastalpha_1 = alpha_1
        derivationFlag = 0

    Xi2_1 = alphadot_1 + (B * k2 * z2_1 * pow(_wd_1,2)) + ((B / A) * k5 * z1_1 * _wd_1)
    
    V_1 = _x3_1 * Xi1_1 + k4 * Xi2_1
    w_1 = Xi1_1

    RPM_Right = ((V_1 + (width*w_1))/r) * 9.55
    RPM_Left = ((V_1 - (width*w_1))/r) * 9.55

    # print(f"V:{V} \tw:{w} \t angle:{angle} \tthetaD:{_thetad}")

    # -------------- Sending Data To MCU
    if not manualControl:
        Send_RPM_to_Robot(RPM_Left, RPM_Right, robot_id)

    # -------------- Record Robot Data
    if time.time() - last_step_time > 0.1:
        derivationFlag = 1
        Xd = _xcd_1*100 + center_x
        Yd = _ycd_1*100 + center_y
        thetaD = _thetad_1
        
        if thetaD < 0: thetaD += 2*pi
        robot_data.append([
            t1, 
            x_Center1, 
            y_Center1, 
            angle_1,
            Xd, 
            Yd,
            x_Center1 - Xd,
            y_Center1 - Yd,
            radians(angle_1),
            thetaD,
            radians(angle_1) - thetaD,
            _x2_1,
            _x3_1,
            _x2d_1,
            _x3d_1
        ])
        last_step_time = time.time()
    
    end = time.time()
    # print((end - start))


    # -------------- Event Handlers
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    key = cv2.waitKey(1)
    # Program Termination
    if key & 0xff == 27:
        Send_RPM_to_Robot(0, 0, 0)
        Send_RPM_to_Robot(0, 0, 1)
        Send_RPM_to_Robot(0, 0, 2)
        break
    if key == ord('s') and not manualControl:
        df = pd.DataFrame(robot_data, columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD', 'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
        try:
            df.to_excel('./recordings/data.xlsx', sheet_name='Robot Positions')
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