import cv2
import numpy as np
import json
import urllib
import urllib.request

url = "http://192.168.18.77:8080/shot.jpg"
# url = "http://172.30.51.82:8080/shot.jpg"
h_lower=0
cameraOrWeb = "camera"
if cameraOrWeb == "camera":

    def readFrame():
        global cap
        global scaling_factor
        cap = cv2.VideoCapture(0)
        scaling_factor = 0.7

    # Capture the input frame from webcam
    def get_frame(cap, scaling_factor):
        global frame
        # Capture the frame from video capture object
        ret, frame = cap.read()

        # Resize the input frame
        frame = cv2.resize(
            frame,
            None,
            fx=scaling_factor,
            fy=scaling_factor,
            interpolation=cv2.INTER_AREA,
        )

        return frame

else:

    def readFrame():
        global scaling_factor
        global cap
        scaling_factor = 0.5
        cap = urllib.request.urlopen(url)

    def get_frame(cap, scaling_factor):
        global frame
        cap = urllib.request.urlopen(url)
        imgNp = np.array(bytearray(cap.read()), dtype=np.uint8)
        frame = cv2.imdecode(imgNp, -1)
        # Resize the input frame
        frame = cv2.resize(
            frame,
            None,
            fx=scaling_factor,
            fy=scaling_factor,
            interpolation=cv2.INTER_AREA,
        )
        return frame


color = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
    "orange": (0, 188, 255),
    "pink": (127, 157, 255),
}

color_select = "red"
with open("data.json", "r") as file:
    data = json.load(file)

v_lower =0
# red
def h_lower_range(range):
    global h_lower
    h_lower = range


def h_upper_range(range):
    global h_upper
    h_upper = range


# green
def s_lower_range(range):
    global s_lower
    s_lower = range


def s_upper_range(range):
    global s_upper
    s_upper = range


# blue
def v_lower_range(range):
    global v_lower
    v_lower = range


def v_upper_range(range):
    global v_upper
    v_upper = range


def hsv_save(color):
    data[color]["lower"]["h"] = h_lower
    data[color]["upper"]["h"] = h_upper
    data[color]["lower"]["s"] = s_lower
    data[color]["upper"]["s"] = s_upper
    data[color]["lower"]["v"] = v_lower
    data[color]["upper"]["v"] = v_upper
    # Specify the file path to save the JSON data
    file_path = "data.json"

    # Write data to the JSON file
    with open(file_path, "w") as file:
        json.dump(data, file)


def setColorBar(color):
    cv2.setTrackbarPos("LR", winname, data[color_select]["lower"]["h"])
    cv2.setTrackbarPos("UR", winname, data[color_select]["upper"]["h"])
    cv2.setTrackbarPos("LG", winname, data[color_select]["lower"]["s"])
    cv2.setTrackbarPos("UG", winname, data[color_select]["upper"]["s"])
    cv2.setTrackbarPos("LB", winname, data[color_select]["lower"]["v"])
    cv2.setTrackbarPos("UB", winname, data[color_select]["upper"]["v"])


# keyboard.on_press(on_key_press)

if __name__ == "__main__":
    readFrame()
    show = "show"
    cv2.namedWindow(show)
    # create trackbar for color change
    winname = "Color Detector"
    cv2.namedWindow(winname)
    # cv2.createButton("set_red", red_save, None, cv2.QT_CHECKBOX, 0)
    # cv2.createTrackbar("Color", winname, 0, 360, color_range)
    cv2.createTrackbar(
        "LR", winname, data[color_select]["lower"]["h"], 255, h_lower_range
    )
    cv2.createTrackbar(
        "UR", winname, data[color_select]["upper"]["h"], 255, h_upper_range
    )
    cv2.createTrackbar(
        "LG", winname, data[color_select]["lower"]["s"], 255, s_lower_range
    )
    cv2.createTrackbar(
        "UG", winname, data[color_select]["upper"]["s"], 255, s_upper_range
    )
    cv2.createTrackbar(
        "LB", winname, data[color_select]["lower"]["v"], 255, v_lower_range
    )
    cv2.createTrackbar(
        "UB", winname, data[color_select]["upper"]["v"], 255, v_upper_range
    )
    # Iterate until the user presses ESC key
    while True:
        frame = get_frame(cap, scaling_factor)
        cv2.putText(
            frame,
            color_select,
            (100, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color[color_select],
        )

        # Convert the HSV colorspace
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only desired color
        # [h, s, v] = colorsys.rgb_to_hsv(red_lower, green_lower, blue_lower)
        lower = np.array([h_lower, s_lower, v_lower])
        # [h, s, v] = colorsys.rgb_to_hsv(red_upper, green_upper, blue_upper)
        upper = np.array([h_upper, s_upper, v_upper])
        mask = cv2.inRange(hsv, lower, upper)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Original image", frame)
      
        cv2.imshow(show, res)
        cv2.imshow(winname, res)

        key = cv2.waitKey(1)

        if key & 0xFF == 27:
            break
        if key == 49:
            color_select = "red"
            setColorBar(color_select)
        if key == 50:
            color_select = "blue"
            setColorBar(color_select)
        if key == 51:
            color_select = "orange"
            setColorBar(color_select)
        if key == 52:
            color_select = "pink"
            setColorBar(color_select)
        if key == 53:
            color_select = "green"
            setColorBar(color_select)
        if key == 54:
            color_select = "yellow"
            setColorBar(color_select)
        if key == 115:
            hsv_save(color_select)
            print("saved")
    cv2.destroyAllWindows()
    #cap.release()
