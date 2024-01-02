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
df = pd.DataFrame([], columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD',
                               'Error Theta', 'x2', 'x3', 'x2d', 'x3d'])
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
x = 0.55
y = 0
angle = 3.14 / 2
Xi1 = 0
Xi2 = 0
lastalpha = 0
alpha = 0
z1 = 0
z2 = 0
alphadot = 0
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
k41 = 5  #####
k42 = 5  #####
V = 0
w = 0
width = 0.125
r = 0.06
R = 0.1375
_xcd = 0
_ycd = 0
_thetad = 0
_wd = 0
_x2d = 0
_x3d = 0
_x2 = 0
_x3 = 0
A = 0
B = 0
speed = 0.1
M_PI = pi
M_2PI = 2 * pi

# --------- Dynamic Variables
# dim1_range = np.linspace(-1.5, 1.5, 4)
# dim2_range = np.linspace(-2, 2, 5)
# dim3_range = np.linspace(0, 2, 3)
# dim4_range = np.linspace(-1.5, 1.5, 4)
# dim5_range = np.linspace(-1, 1, 3)
# dim6_range = np.linspace(0, 2, 3)
dim1_range = np.linspace(-2, 2, 4)  ######
dim2_range = np.linspace(-5, 5, 5)  ######
dim3_range = np.linspace(-0.5, 0.5, 3)  ######
dim4_range = np.linspace(-0.5, 0.5, 4)  ######
dim5_range = np.linspace(-1, 1, 3)
dim6_range = np.linspace(0, 2, 3)
centers = np.array(list(product(dim1_range, dim2_range, dim3_range, dim4_range, dim5_range, dim6_range)))
Xi11_dot = 0
Xi12_dot = 0
x13_dot = 0

last_Xi11 = Xi1
last_Xi12 = Xi2

Xi1_Actual1 = 0
Xi2_Actual1 = 0

last_Xi1_Actual1 = Xi1_Actual1
last_Xi2_Actual1 = Xi2_Actual1

last_x3 = _x3
XiVirtual1 = [Xi1, Xi2]
Xidotvirtual1 = [Xi11_dot, Xi12_dot]
XidotActual1 = [0, 0]
XiActual1 = np.zeros((2, 1), float)

Xi1_Actual1 = 0
Xi2_Actual1 = 0

XidotActual1 = np.zeros((2, 1), float)
neurons = 200
PHIvec1 = np.zeros((neurons, 1), float)
z13 = np.array([[0], [0]])
U1 = 0
u1 = np.array([[0], [0]])
beta1 = np.array([[1 / r], [R / r]])
GAMAofbeta = np.array([[0.000001, 0], [0, 0.000005]])
dt = 0.01
GAMAofW = np.zeros((neurons, neurons), int)
np.fill_diagonal(GAMAofW, 50)
Kv1 = np.zeros((2, 2), int)  ######
np.fill_diagonal(Kv1, 100)
W1 = np.zeros((neurons, 2), float)
NNoutput1 = np.zeros((2, 1))
ks1 = 0.1
le31 = 1.5
le32 = 1.2
k41 = 2
k42 = 2
ng = 60.5  # 10
kt = 0.2639
kb = 0.019
ra = 27.0
ku1 = (ng * kt) / ra
ku2 = ng * kb * ku1
m = 4.3
J = 5


# --------- Math Additional Functions
def constrainAngle(x):
    x = fmod(x + M_PI, M_2PI)
    if (x < 0):
        x += M_2PI
    return x - M_PI


def angleConv(angle):
    return fmod(constrainAngle(angle), M_2PI)


def angleDiff(a, b):
    dif = fmod(b - a + M_PI, M_2PI)
    if (dif < 0):
        dif += M_2PI
    return dif - M_PI


def unwrap(previousAngle, newAngle):
    return previousAngle - angleDiff(newAngle, angleConv(previousAngle))


def sin2(val):
    return pow(sin(val), 2)


def cos2(val):
    return pow(cos(val), 2)


def absf(input):
    if (input >= 0): return input
    return -input


# --------- Kinematic Functions
def xcd():
	return 0.30 * cos(t1*speed)
def ycd():
	return 0.45 * sin(t1*speed)
def thetad():
	theta = atan2(0.45*cos(t1*speed), -0.30*sin(t1*speed))
	return unwrap(_thetad, theta)
def wd():
	return (6*speed * (sin2(t1*speed) +   cos2(t1*speed))) /	(4*sin2(t1*speed) + 9*cos2(t1*speed))
def x2d():
	return (_xcd * cos(_thetad)) + (_ycd * sin(_thetad))
def x3d():
	return (_xcd * sin(_thetad)) - (_ycd * cos(_thetad))
def x2(xc, yc):
	return (xc * cos(angle)) + (yc * sin(angle))
def x3(xc, yc):
	return (xc * sin(angle)) - (yc * cos(angle))


# --------- Dynamic Function
def PHI(inputVec, C):
    squared_distance = np.linalg.norm(inputVec - C)
    a = exp(-squared_distance / (1.4 ** 2))
    return a


with open("data.json", "r") as openfile:
    hsvColors = json.load(openfile)
webcam = cv2.VideoCapture(0)
ser = [
    serial.Serial(timeout=0.1),
    serial.Serial(timeout=0.1),
    serial.Serial(timeout=0.1),
]
ser[0].baudrate = 115200
ser[1].baudrate = 115200
ser[2].baudrate = 115200
ser[0].port = 'COM10'
ser[1].port = 'COM11'
ser[2].port = 'COM18'
robot_id = 0

try:
    ser[0].open()
except:
    pass
try:
    ser[1].open()
except:
    pass
try:
    ser[2].open()
except:
    pass

packet_green = [0, 0, 0, 0, 0, 0]
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


def Send_RPM_to_Robot(u1_1, u1_2, robot_id):  # (right, left)
    global packet_green, XiActual1, t1, Xi1_Actual1, Xi2_Actual1
    if u1_1 > 30:
        u1_1 = 30
    if u1_2 > 30:
        u1_2 = 30
    if u1_1 < -30:
        u1_1 = -30
    if u1_2 < -30:
        u1_2 = -30

    if u1_1 >= 0:
        packet_green[0] = 0
    else:
        packet_green[0] = 1
    u1_1 *= 100
    packet_green[1] = int(abs(u1_1)).to_bytes(2, "little", signed=True)[0]
    packet_green[2] = int(abs(u1_1)).to_bytes(2, "little", signed=True)[1]
    if u1_2 >= 0:
        packet_green[3] = 0
    else:
        packet_green[3] = 1
    u1_2 *= 100
    packet_green[4] = int(abs(u1_2)).to_bytes(2, "little", signed=True)[0]
    packet_green[5] = int(abs(u1_2)).to_bytes(2, "little", signed=True)[1]
    try:
        ser[robot_id].open()
        ser[robot_id].write(packet_green)
    except:
        print(f'Robot {robot_id + 1} is not connected !!')
    if not manualControl:
        data = ser[robot_id].read(6)
        w_rec = 0
        V_rec = 0
        if len(data) >= 6:
            w_rec = (data[1] | (data[2] << 8)) / 100.0
            V_rec = (data[4] | (data[5] << 8)) / 100.0
            if data[0] == 1:
                w_rec = -w_rec
            if data[3] == 1:
                V_rec = -V_rec
        Xi1_Actual1 = w_rec
        Xi2_Actual1 = V_rec - _x3 * w_rec
        XiActual1 = np.array([[Xi1_Actual1], [Xi2_Actual1]])
    try:
        ser[robot_id].close()
    except:
        pass
    # print(f"V: {V_rec}   w: {w_rec}")


while 1:
    start = time.time()
    ret, imageFrame = webcam.read()  # cv2.imdecode(imgNp, -1)
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsvFrame, colorSetHSV("red", "lower"), colorSetHSV("red", "upper"))
    green_mask = cv2.inRange(hsvFrame, colorSetHSV("green", "lower"), colorSetHSV("green", "upper"))
    blue_mask = cv2.inRange(hsvFrame, colorSetHSV("blue", "lower"), colorSetHSV("blue", "upper"))
    yellow_mask = cv2.inRange(hsvFrame, colorSetHSV("yellow", "lower"), colorSetHSV("yellow", "upper"))
    orange_mask = cv2.inRange(hsvFrame, colorSetHSV("orange", "lower"), colorSetHSV("orange", "upper"))
    pink_mask = cv2.inRange(hsvFrame, colorSetHSV("pink", "lower"), colorSetHSV("pink", "upper"))
    kernal = np.ones((5, 5), "uint8")
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)
    green_mask = cv2.dilate(green_mask, kernal)
    green_mask = cv2.erode(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)
    blue_mask = cv2.dilate(blue_mask, kernal)
    blue_mask = cv2.erode(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    # -------------- red
    contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 50 and area < 50000:
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
        if area > 50 and area < 50000:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_green, y_green, w_green, h_green = cv2.boundingRect(contour)
            if w_green > 200 or h_green > 200:
                break
            cv2.drawContours(imageFrame, [box], 0, (0, 255, 0), 2)

            x_green_center = (x_green + (w_green / 2))
            y_green_center = (y_green + (h_green / 2))

            x_Center1 = round(x_red_center * 0.489, 2)
            y_Center1 = round((480 - y_red_center) * 0.489, 2)

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
        if area > 600 and area < 900:
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
    x = (x_Center1 - center_x) / 100
    y = (y_Center1 - center_y) / 100
    angle = unwrap(angle, radians(green_angle))
    t1 = time.time() - beginTime
    _xcd = xcd()
    _ycd = ycd()
    _thetad = thetad()
    _wd = wd()
    _x2d = x2d()
    _x3d = x3d()
    _x2 = x2(x, y)
    _x3 = x3(x, y)

    Xi1 = _wd + (k3 * (_thetad - angle))
    z1 = _x3d - _x3
    A = pow(le1, 2) - pow(z1, 2)
    alpha = _x2d + (A * k1 * z1 * _wd)
    z2 = alpha - _x2
    B = pow(le2, 2) - pow(z2, 2)

    # if(derivationFlag):
    #     alphadot = (alpha - lastalpha)/0.1
    #     lastalpha = alpha
    alphadot = (alpha - lastalpha) / dt
    Xi2 = alphadot + (B * k2 * z2 * pow(_wd, 2)) + ((B / A) * k5 * z1 * _wd)
    # if(derivationFlag):
    #     Xi11_dot = (Xi1 - last_Xi11)/0.1
    #     Xi12_dot = (Xi2 - last_Xi12)/0.1
    #     x13_dot  = (_x3 - last_x3 )/0.1
    #     last_Xi11 = Xi1
    #     last_Xi12 = Xi2
    #     last_x3 = _x3
    #     derivationFlag = 0
    Xi11_dot = (Xi1 - last_Xi11) / dt
    Xi12_dot = (Xi2 - last_Xi12) / dt

    Xi11_dot_act = (Xi1_Actual1 - last_Xi1_Actual1) / dt
    Xi12_dot_act = (Xi2_Actual1 - last_Xi2_Actual1) / dt

    x13_dot = (_x3 - last_x3) / dt
    XiVirtual1 = np.array([[Xi1], [Xi2]])
    Xidotvirtual1 = np.array([[Xi11_dot], [Xi12_dot]])
    XidotActual1 = np.array([[Xi11_dot_act], [Xi12_dot_act]])

    inputVec1 = np.array([Xi1, Xi2, Xi11_dot, Xi12_dot, x13_dot, _x3])

    z13 = XiVirtual1 - XiActual1
    U1 = [[u1[0][0] + u1[1][0], 0],
          [0, u1[0][0] - u1[1][0]]]
    beta1 = beta1 + dt * np.matmul(np.matmul(GAMAofbeta, U1), z13)
    es_r1 = 1 / beta1[0][0]
    es_R1 = es_r1 * beta1[1][0]

    B1hat1 = np.array([[(_x3 + es_R1) / es_r1, (_x3 - es_R1) / es_r1],
                       [1 / es_r1, 1 / es_r1]])
    B1 = np.array([[(_x3 + R) / r, (_x3 - R) / r],
                   [1 / r, 1 / r]])
    for i in range(neurons):
        PHIvec1[i] = PHI(inputVec1, centers[i])
    PHIvec1 = np.array(PHIvec1)

    W1 = W1 + dt * (np.matmul(np.matmul(GAMAofW, PHIvec1), z13.transpose()))  # - (RHO * ((W1 - W2)))
    NNoutput1 = np.matmul(W1.transpose(), PHIvec1)
    u1 = np.matmul(np.linalg.inv(B1),
                   (NNoutput1 + np.matmul(Kv1, z13) + ks1 * np.sign(z13) + (
                           (np.linalg.pinv(z13.transpose())) * (
                           (z13[0][0] * (Xidotvirtual1[0][0] - XidotActual1[0][0]) / (le31 ** 2 - z13[0][0] ** 2)) +
                           (z13[1][0] * (Xidotvirtual1[1][0] - XidotActual1[1][0]) / (le32 ** 2 - z13[1][0] ** 2)) +
                           ((k41 * z13[0][0] ** 2) / (le31 ** 2 - z13[0][0] ** 2)) +
                           ((k42 * z13[1][0] ** 2) / (le32 ** 2 - z13[1][0] ** 2))
                   )
                   )))
    # print(u1)
    # XiActual1 =  XiActual1 + dt * np.matmul(np.linalg.inv((1/ku1) * np.array([[m * _x3**2 + J, m * _x3], [m * _x3, m]])) , (np.matmul(np.array([[(_x3 + R)/r, (_x3 - R)/r], [1/r, 1/r]]), u1) - (1/ku1) * np.matmul(np.array([[m * _x3 * x13_dot, 0], [m * x13_dot, 0]]), XiActual1) - ((2*ku2)/(ku1*r**2)) * np.matmul(np.array([[_x3**2 + R**2, _x3], [_x3, 1]]), XiActual1) - np.matmul(np.array([[_x3, 1], [1, 0]]), np.array([[30 * V + 4 * np.sign(V)], [30 * w + 4 * np.sign(w)]])) - (1/ku1) * np.matmul(np.array([[_x3, 1], [1, 0]]), np.array([[0.1*sin(t1)], [0.1*cos(t1)]]))))
    lastalpha = alpha
    last_Xi11 = Xi1
    last_Xi12 = Xi2

    last_Xi1_Actual1 = Xi1_Actual1
    last_Xi2_Actual1 = Xi2_Actual1

    last_x3 = _x3
    # -------------- Prints:

    print(f"u1:{round(u1[0][0], 2)}\t u2:{round(u1[1][0], 2)}\t")
    # print(f"V:{V}\t w:{w}\t theta:{round(angle, 2)}\t thetaD:{round(_thetad, 2)}\t dt:{round(dt, 2)}\t u1[0]:{round(u1[0][0], 2)}\t u1[1]:{round(u1[1][0],2)}")
    # print(f"z1:{round(z1, 2)}  \tz2:{round(z2, 2)}  \tz13:[{round(z13[0][0], 2)}, {round(z13[1][0], 2)}]")
    # print(f"Xidotvirtual1:[{round(Xidotvirtual1[0][0],2)}, {round(Xidotvirtual1[1][0],2)}] \t XiVirtual1:[{round(XiVirtual1[0][0],2)}, {round(XiVirtual1[1][0],2)}]")

    # -------------- Sending Data To MCU
    if not manualControl:
        Send_RPM_to_Robot(u1[0][0], u1[1][0], robot_id)

    end = time.time()
    dt = end - start

    # -------------- Record Robot Data
    if time.time() - last_step_time > 0.1:
        derivationFlag = 1
        Xd = _xcd * 100 + center_x
        Yd = _ycd * 100 + center_y
        thetaD = _thetad

        if thetaD < 0:
            thetaD += 2 * pi
        robot_data.append([
            t1,
            x_Center1,
            y_Center1,
            angle,
            Xd,
            Yd,
            x_Center1 - Xd,
            y_Center1 - Yd,
            radians(green_angle),
            thetaD,
            radians(green_angle) - thetaD,
            _x2,
            _x3,
            _x2d,
            _x3d,
            NNoutput1[0][0],
            NNoutput1[1][0]
        ])
        last_step_time = time.time()

    # print()

    # -------------- Event Handlers
    imageFrame = cv2.ellipse(imageFrame, (int((center_x-90)/0.489), int(480 - center_y/0.489)), (int(30/0.489), int(45/0.489)), 0, 0, 360, (0, 188, 255), 2) 
    imageFrame = cv2.ellipse(imageFrame, (int(center_x/0.489), int(480 - center_y/0.489)), (int(30/0.489), int(45/0.489)), 0, 0, 360, (255, 0, 0), 2) 
    imageFrame = cv2.ellipse(imageFrame, (int((center_x+90)/0.489), int(480 - center_y/0.489)), (int(30/0.489), int(45/0.489)), 0, 0, 360, (0, 0, 255), 2) 
    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    key = cv2.waitKey(1)
    # Program Termination
    if key & 0xff == 27:
        Send_RPM_to_Robot(0, 0, robot_id)
        break
    if key == ord('s') and not manualControl:
        df = pd.DataFrame(robot_data,
                          columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd', 'Error X', 'Error Y', 'Theta Rad', 'ThetaD',
                                   'Error Theta', 'x2', 'x3', 'x2d', 'x3d', 'NN(0)', 'NN(1)'])
        try:
            df.to_excel('./recordings/data.xlsx', sheet_name='Robot Positions')
            print('Excel Saved')
            Send_RPM_to_Robot(0, 0)
            break
        except:
            print('Can not save file. Permission denied !!!!!!!')
    if key == ord('w') and manualControl:
        Send_RPM_to_Robot(12, 12, robot_id)
    if key == ord('s') and manualControl:
        Send_RPM_to_Robot(-12, -12, robot_id)
    if key == ord('a') and manualControl:
        Send_RPM_to_Robot(12, -12, robot_id)
    if key == ord('d') and manualControl:
        Send_RPM_to_Robot(-12, 12, robot_id)
    if key == ord(' ') and manualControl:
        Send_RPM_to_Robot(0, 0, robot_id)
    if key == ord('m'):
        manualControl = not manualControl
        Send_RPM_to_Robot(0, 0, robot_id)
    if key == ord('1'):
        robot_id = 0
    if key == ord('2'):
        robot_id = 1
    if key == ord('3'):
        robot_id = 2

# cap.release()
cv2.destroyAllWindows()
