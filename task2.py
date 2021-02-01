import math
import rospy
import cv2
import numpy as np
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

cur_col = ""

arr = [[0.0, 2.5], [3.5, 0.5], [2.0, 1.5], [3.5, 3.5]]
colours = ["", "", "", ""]

bridge = CvBridge()

rospy.init_node('task2')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0.0, y=0.0, z=0.5, yaw=float('nan'), speed=0.4, frame_id='map', auto_arm=False, tolerance=0.3):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.1)

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.1)

def take_off():
	navigate_wait(x=0.0, y=0.0, z=0.5, yaw=0.0, speed=0.4, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(2)	

def nextMark(xc, yc):
	navigate_wait(x=float(xc), y=float(yc), z=0.5, yaw=0.0, speed=0.4, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def home():
	navigate_wait(x=0.0, y=0.0, z=0.5, yaw=0.0, speed=0.4, frame_id='map',  auto_arm=True, tolerance=0.1)
	rospy.sleep(1)

def image_callback(data):

	global cur_col

	cv2.startWindowThread()
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	cv_image = cv_image[48:192, 64:256]
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

	lower_green = np.array([55, 100, 0])
        upper_green = np.array([65, 255, 255])
	mask_green = cv2.inRange(hsv, lower_green, upper_green)
	res_green = cv2.bitwise_and(hsv, hsv, mask=mask_green)
	res_green = cv2.cvtColor(res_green, cv2.COLOR_BGR2GRAY)
	ret, res_green = cv2.threshold(res_green, 127, 255, 0)
	cv2.namedWindow("green")
	cv2.imshow("green", res_green)

	lower_yellow = np.array([25, 100, 0])
        upper_yellow = np.array([35, 255, 255])
	mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
	res_yellow = cv2.bitwise_and(hsv, hsv, mask=mask_yellow)
	res_yellow = cv2.cvtColor(res_yellow, cv2.COLOR_BGR2GRAY)
	ret, res_yellow = cv2.threshold(res_yellow, 127, 255, 0)
	cv2.namedWindow("yellow")
	cv2.imshow("yellow", res_yellow)

	lower_red1 = np.array([0, 100, 0])
        upper_red1 = np.array([5, 255, 255])

	lower_red2 = np.array([175, 100, 0])
        upper_red2 = np.array([179, 255, 255])
	mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
	res_red1 = cv2.bitwise_and(hsv, hsv, mask=mask_red1)
	res_red2 = cv2.bitwise_and(hsv, hsv, mask=mask_red2)
	res_red = cv2.bitwise_or(res_red1, res_red2)
	res_red = cv2.cvtColor(res_red, cv2.COLOR_BGR2GRAY)
	ret, res_red = cv2.threshold(res_red, 127, 255, 0)
	cv2.namedWindow("red")
	cv2.imshow("red", res_red)

	_, green_contours, _ = cv2.findContours(res_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(green_contours) == 1:
		print("green")
		cur_col = "green"

	_, yellow_contours, _ = cv2.findContours(res_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(yellow_contours) == 1:
		print("yellow")
		cur_col = "yellow"

	_, red_contours, _ = cv2.findContours(res_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(red_contours) == 1:
		print("red")
		cur_col = "red"

	
def main():

	image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)

	take_off()
	for i in range(0, 4):
		nextMark(arr[i][0], arr[i][1])
		colours[i] = cur_col
	home()
	land_wait()
	cv2.destroyAllWindows()
	file = open("results.txt", "w")
	for i in range(0, 4):
		file.write("x = " + str(arr[i][0]) + ", y = " + str(arr[i][1]) + ": " + colours[i] + "\n")
		print("x = " + str(arr[i][0]) + ", y = " + str(arr[i][1]) + ": " + colours[i] + "\n")
	file.close()

main()
