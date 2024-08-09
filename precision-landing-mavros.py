import rospy
from sensor_msgs.msg import Image
import cv2
import time
import numpy as np
from cv2 import aruco
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import argparse

class MarkerDetector:
    def __init__(self, target_type, target_size, camera_info):
        self.target_type = target_type
        self.marker_size = target_size

        if self.target_type == 'aruco':
            self.dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
            self.parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        elif self.target_type == 'qrcode':
            print("QR Code not implemented yet!")

        self.camera_matrix = camera_info[0]
        self.dist_coeff = camera_info[1]

        self.np_camera_matrix = np.array(self.camera_matrix)
        self.np_dist_coeff = np.array(self.dist_coeff)

        self.horizontal_res = camera_info[2][0]
        self.vertical_res = camera_info[2][1]

        self.horizontal_fov = camera_info[3][0]
        self.vertical_fov = camera_info[3][1]

    def pose_estimation(self, corners, marker_size, mtx, distortion):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, marker_size / 2, 0],
                                  [marker_size / 2, -marker_size / 2, 0],
                                  [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

        nada, rvec, tvec = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        return rvec, tvec

    def aruco_detection(self, frame):
        markerCorners, markerIds, rejected = self.detector.detectMarkers(frame)

        i = 0
        if len(markerCorners) > 0:
            closest_target = []
            closest_dist = 100000

            for corners in markerCorners:
                marker_points = corners[0]
                final_image = self.draw_marker(frame, marker_points)
                pose = self.pose_estimation(marker_points, self.marker_size, self.np_camera_matrix, self.np_dist_coeff)
                rvec, tvec = pose
                x = round(tvec[0][0], 2)
                y = round(tvec[1][0], 2)
                z = round(tvec[2][0], 2)
                x_sum = marker_points[0][0] + marker_points[1][0] + marker_points[2][0] + marker_points[3][0]
                y_sum = marker_points[0][1] + marker_points[1][1] + marker_points[2][1] + marker_points[3][1]
                x_avg = x_sum / 4
                y_avg = y_sum / 4
                x_ang = (x_avg - self.horizontal_res * 0.5) * self.horizontal_fov / self.horizontal_res
                y_ang = (y_avg - self.vertical_res * 0.5) * self.vertical_fov / self.vertical_res
                payload = markerIds[i][0]
                i += 1

                if z < closest_dist:
                    closest_dist = z
                    closest_target = [x, y, z, x_ang, y_ang, payload, final_image]

            return closest_target
        return None

    def draw_marker(self, frame, points):
        topLeft, topRight, bottomRight, bottomLeft = points
        tR = (int(topRight[0]), int(topRight[1]))
        bR = (int(bottomRight[0]), int(bottomRight[1]))
        bL = (int(bottomLeft[0]), int(bottomLeft[1]))
        tL = (int(topLeft[0]), int(topLeft[1]))
        cX = int((tR[0] + bL[0]) / 2.0)
        cY = int((tR[1] + bL[1]) / 2.0)
        rect = cv2.rectangle(frame, tL, bR, (0, 0, 255), 2)
        final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)
        return final


class PrecLand:
    def __init__(self, target_type, target_size, camera_info):
        self.detector = MarkerDetector(target_type, target_size, camera_info)
        rospy.init_node('drone_node', anonymous=False)
        self.bridge_object = CvBridge()
        self.newimg_pub = rospy.Publisher('camera/colour/image_new', Image, queue_size=10)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)
        self.local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.current_state = State()
        self.teste = []
        self.cam = cv2.VideoCapture(0)

    def state_cb(self, state):
        self.current_state = state

    def send_land_message(self, x_ang, y_ang, dist_m, time=0):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = x_ang
        msg.pose.position.y = y_ang
        msg.pose.position.z = dist_m
        self.local_pos_pub.publish(msg)
        print("Mensagem enviada")

    def msg_receiver(self):
        ret, frame = self.cam.read()
        if not ret:
            return

        closest_target = self.detector.aruco_detection(frame)
        if closest_target is not None and self.current_state.mode == 'LAND':
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target
            dist = float(z) / 100
            self.send_land_message(x_ang, y_ang, dist)
            print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
            ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
            self.newimg_pub.publish(ros_img)

    def arm_and_takeoff(self, aTargetAltitude):
        while not self.current_state.connected:
            print(" Waiting for vehicle to connect...")
            time.sleep(1)

        self.set_mode_client(base_mode=0, custom_mode="GUIDED")
        self.arming_client(True)

        while not self.current_state.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        pose = PoseStamped()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = aTargetAltitude

        for i in range(100):
            self.local_pos_pub.publish(pose)
            time.sleep(0.1)

        while not rospy.is_shutdown():
            self.local_pos_pub.publish(pose)
            if self.current_state.mode == "GUIDED" and self.current_state.armed:
                print("Taking off!")
                break
            time.sleep(1)

        while self.current_state.mode == "GUIDED":
            print(" Altitude: ", self.current_state.altitude.local)
            if self.current_state.altitude.local >= aTargetAltitude * 0.95:
                print(f"Reached target altitude {self.current_state.altitude.local}")
                break
            time.sleep(1)


if __name__ == '__main__':
    marker_size = 50
    camera_matrix = [[467.74270306499267, 0.0, 320.5],
                     [0.0, 467.74270306499267, 240.5],
                     [0.0, 0.0, 1.0]]
    dist_coeff = [0.0, 0.0, 0.0, 0.0, 0]
    res = (640, 480)
    fov = (1.2, 1.1)
    camera = [camera_matrix, dist_coeff, res, fov]

    precision_landing = PrecLand('aruco', marker_size, camera)

    precision_landing.arm_and_takeoff(10)
    print("Take off complete")

    if precision_landing.current_state.mode != 'LAND':
        precision_landing.set_mode_client(base_mode=0, custom_mode='LAND')
        while precision_landing.current_state.mode != 'LAND':
            time.sleep(1)
        print('vehicle in LAND mode')

    print("Going for precision landing...")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        precision_landing.msg_receiver()
        rate.sleep()

    print("END")
