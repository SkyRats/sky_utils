import rospy
from geometry_msgs.msg import Point, Vector3
from dronekit import VehicleMode
from pymavlink import mavutil

class PrecisionLanding:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.pose = Point()
        self.angle = Vector3()

        # ROS subscribers to listen for target pose and angle
        rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.update_pose,queue_size=1,buff_size=2**24)
        rospy.Subscriber('/sky_vision/down_cam/aruco/angle', Vector3, self.update_angle,queue_size=1,buff_size=2**24)

        # @self.vehicle.on_message('GLOBAL_POSITION_INT')
        # def listener(_, name, message):
        #     self.current_lat = message.lat
        #     self.current_lon = message.lon
        #     self.current_alt = message.relative_alt / 1000.0

    def update_pose(self, msg):
        self.pose = msg

    def update_angle(self, msg):
        self.angle = msg

    def send_land_message(self, x_ang, y_ang, dist_m, time=0):
        msg = self.vehicle.message_factory.landing_target_encode(
            time,
            0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            x_ang,
            y_ang,
            dist_m,
            0,
            0,
        )
        self.vehicle.send_mavlink(msg)
        print("Landing message sent")


    def precision_landing(self):

        # Use the received ArUco information
        if self.pose and self.angle:
            x_ang = self.angle.x
            y_ang = self.angle.y
            dist = float(self.pose.z) / 100

            print("Altitude:", self.vehicle.location.global_relative_frame.alt)

            if self.vehicle.mode != 'LAND':
                self.vehicle.mode = VehicleMode('LAND')
                while self.vehicle.mode != 'LAND':
                    rospy.sleep(0.1)
                print('Vehicle in LAND mode')

            self.send_land_message(x_ang, y_ang, dist)

            print(f'MARKER POSITION: x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | z = {self.pose.z}')
