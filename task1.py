import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from pyzbar import pyzbar
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

x = 0.0
y = 0.0

bridge = CvBridge()

rospy.init_node('task1')
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x=0.0, y=0.0, z=0.5, yaw=float('nan'), speed=0.4, frame_id='map', auto_arm=False, tolerance=0.2):
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
	navigate_wait(x=float(xc), y=float(yc), z=0.5, yaw=0.0, speed=0.4, frame_id='map',  auto_arm=True, tolerance=0.2)
	rospy.sleep(1)

def image_callback(data):
	global x
	global y
	cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
	cv_image = cv_image[48:192, 64:256]
	barcodes = pyzbar.decode(cv_image)
	for barcode in barcodes:
		b_data = barcode.data.encode("utf-8")
		x = b_data[0]
		y = b_data[2]
		print ("next point: x={}, y={}".format(b_data[0], b_data[2]))


def main():
	image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback, queue_size=1)
	global x
	global y
	take_off()
	nextMark(2.0, 0.0)
	nextMark(x, y)
	nextMark(x, y)
	nextMark(x, y)
	land_wait()
main()
