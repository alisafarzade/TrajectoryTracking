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

df = pd.DataFrame([], columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd'])
robot_data = []
beginTime = time.time()
last_step_time = time.time()
circle_x = 140
circle_y = 130
Line_x = 35
Line_y = 145
with open("data.json", "r") as openfile:
    # Reading from json file
    hsvColors = json.load(openfile)
# Capturing video through webcam
webcam = cv2.VideoCapture(0)

ser = serial.Serial()
ser.baudrate = 115200
ser.port = 'COM22'

url = "http://192.168.18.77:8080/shot.jpg"

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


# Start a while loop
while 1:
    # Reading the video from the
    # webcam in image frames
    # _, imageFrame = webcam.read()

    start = time.time()

    # imgResp = urllib.request.urlopen(url)
    # imgNp = np.array(bytearray(imgResp.read()), dtype=np.uint8)
    ret, imageFrame = webcam.read()#cv2.imdecode(imgNp, -1)

    # Convert the imageFrame in
    # BGR(RGB color space) to
    # HSV(hue-saturation-value)
    # color space
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color and
    # define mask
    # red_lower = np.array([0, 165, 169], np.uint8)
    # red_upper = np.array([180, 255, 255], np.uint8)

    red_mask = cv2.inRange(
        hsvFrame, colorSetHSV("red", "lower"), colorSetHSV("red", "upper")
    )
    
    # Set range for green color and
    # define mask
    # green_lower = np.array([25, 52, 72], np.uint8)
    # green_upper = np.array([102, 255, 255], np.uint8)

    green_mask = cv2.inRange(
        hsvFrame, colorSetHSV("green", "lower"), colorSetHSV("green", "upper")
    )

    # Set range for blue color and
    # define mask

    blue_mask = cv2.inRange(
        hsvFrame, colorSetHSV("blue", "lower"), colorSetHSV("blue", "upper")
    )

    # Set range for yellow color and
    # define mask

    yellow_mask = cv2.inRange(
        hsvFrame, colorSetHSV("yellow", "lower"), colorSetHSV("yellow", "upper")
    )

    # Set range for orange color and
    # define mask

    orange_mask = cv2.inRange(
        hsvFrame, colorSetHSV("orange", "lower"), colorSetHSV("orange", "upper")
    )

    # Set range for pink color and
    # define mask

    pink_mask = cv2.inRange(
        hsvFrame, colorSetHSV("pink", "lower"), colorSetHSV("pink", "upper")
    )

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernal)
    # red_mask = cv2.erode(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

    # For green color
    green_mask = cv2.dilate(green_mask, kernal)
    green_mask = cv2.erode(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

    # For blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    blue_mask = cv2.erode(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(
        red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 50 and area<50000:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_red, y_red, w_red, h_red = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 0, 255), 2)
            # red_angle = int(rect[2])
            # getOrientation(contour, imageFrame)

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

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(
        green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
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
            # green_angle = int(rect[2])
            # getOrientation(contour, imageFrame)
            
            
            x_green_center = (x_green + (w_green / 2))
            y_green_center = (y_green + (h_green / 2))
            
            # Center of robot (cm)
            
            #x_Center1 = (x_red_center + x_green_center) / 2 - x_centerPoint
            #y_Center1 = (y_red_center + y_green_center) / 2 - y_centerPoint
            x_Center1 = round(((x_red_center + x_green_center) / 2) * 0.489 , 2)
            y_Center1 = round(((y_red_center + y_green_center) / 2) * 0.489 , 2)            
            
            green_angle = degrees(
                atan2((y_green_center - y_red_center), (x_green_center - x_red_center))
            )
            if green_angle < 0:
                green_angle = green_angle + 360

            cv2.putText(
                imageFrame,
                "Green "
                + str(x_Center1)
                + " "
                + str(y_Center1)
                + " "
                + str(int(green_angle)),
                (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
            )

            packet_green[0] = int(x_Center1).to_bytes(2, "little",signed=True)[0]
            packet_green[1] = int(x_Center1).to_bytes(2, "little",signed=True)[1]

            packet_green[2] = int(y_Center1).to_bytes(2, "little",signed=True)[0]
            packet_green[3] = int(y_Center1).to_bytes(2, "little",signed=True)[1]

            packet_green[4] = int(green_angle).to_bytes(2, "little")[0]
            packet_green[5] = int(green_angle).to_bytes(2, "little")[1]
            if time.time() - last_step_time > 0.1:
                df2 = pd.DataFrame([[time.time() - beginTime, x_Center1, y_Center1, green_angle]], columns=['time', 'X', 'Y', 'theta'])
                robot_data.append([
                    time.time() - beginTime, 
                    x_Center1, 
                    y_Center1, 
                    green_angle,
                    30 * cos(time.time() - beginTime) + circle_x, 
                    30 * sin(time.time() - beginTime) + circle_y])
                last_step_time = time.time()

            
    ser.open()
    ser.write(packet_green)
    ser.close()
    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(
        blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
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

    # Creating contour to track yellow color
    contours, hierarchy = cv2.findContours(
        yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

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

    # Creating contour to track orange color
    contours, hierarchy = cv2.findContours(
        orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_orange, y_orange, w_orange, h_orange = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (0, 188, 255), 2)
            # red_angle = int(rect[2])
            # getOrientation(contour, imageFrame)

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

    # Creating contour to track pink color
    contours, hierarchy = cv2.findContours(
        pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_pink, y_pink, w_pink, h_pink = cv2.boundingRect(contour)
            cv2.drawContours(imageFrame, [box], 0, (127, 157, 255), 2)
            # red_angle = int(rect[2])
            # getOrientation(contour, imageFrame)

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
    end = time.time()
    # print((end - start))

    cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
    key = cv2.waitKey(1)
    # Program Termination
    if key & 0xff == 27:
        break
    if key == 115:
        if len(robot_data) > 0:
            dt = (2*pi)/len(robot_data)
            for i in range(len(robot_data)):
                # robot_data[i][4] = cos(i*dt) * 20 + circle_x
                # robot_data[i][5] = sin(i*dt) * 20 + circle_y
                # robot_data[i][4] = i*2 + Line_x
                # robot_data[i][5] = i*2 + Line_y
                pass
                
        df = pd.DataFrame(robot_data, columns=['time', 'X', 'Y', 'theta', 'Xd', 'Yd'])
        df.to_excel('./recordings/data.xlsx', sheet_name='Robot Positions')
        print('Excel Saved')
    if key == 114:
        robot_data = []
        beginTime = time.time()
        print('Recording Started!!')
#cap.release()
cv2.destroyAllWindows()