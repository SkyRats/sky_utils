import dronekit
from dronekit import connect, VehicleMode
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import argparse
import numpy as np
from pymavlink import mavutil


class PrecLand:

    def __init__(self, detector, simulation):
        self.simulation = simulation
        parser = argparse.ArgumentParser()
        parser.add_argument('--connect', default='tcp:127.0.0.1:5763' if simulation else 'tcp:127.0.0.1:5763')
        args = parser.parse_args()
        self.do_plnd = False

        # Connect to vehicle
        print('Connecting...')
        self.vehicle = connect(args.connect, wait_ready=True)

        self.commands = self.vehicle.commands
        #-- Check vehicle status
        print(f"Mode: {self.vehicle.mode.name}")
        print(" Global Location: %s" % self.vehicle.location.global_frame)
        print(" Global Location (relative altitude): %s" % self.vehicle.location.global_relative_frame)
        print(" Local Location: %s" % self.vehicle.location.local_frame)
        print(" Attitude: %s" % self.vehicle.attitude)
        print(" Velocity: %s" % self.vehicle.velocity)
        print(" Gimbal status: %s" % self.vehicle.gimbal)
        print(" EKF OK?: %s" % self.vehicle.ekf_ok)
        print(" Last Heartbeat: %s" % self.vehicle.last_heartbeat)
        print(" Rangefinder: %s" % self.vehicle.rangefinder)
        print(" Rangefinder distance: %s" % self.vehicle.rangefinder.distance)
        print(" Rangefinder voltage: %s" % self.vehicle.rangefinder.voltage)
        print(" Is Armable?: %s" % self.vehicle.is_armable)
        print(" System status: %s" % self.vehicle.system_status.state)
        print(" Armed: %s" % self.vehicle.armed)    # settable
        self.landed = False
        self.detector = detector

    def send_land_message(self, x_ang, y_ang, dist_m):
        msg = self.vehicle.message_factory.landing_target_encode(
            0, 0, mavutil.mavlink.MAV_FRAME_BODY_FRD, x_ang, y_ang, dist_m, 0, 0
        )
        self.vehicle.send_mavlink(msg)
        print("Landing message sent.")

    def switch_to_land_mode(self):
        """Ensure the vehicle is in LAND mode."""
        if self.vehicle.mode != 'LAND':
            self.vehicle.mode = VehicleMode('LAND')
            while self.vehicle.mode != 'LAND':
                rospy.sleep(0.1)  # Allow time for mode switch
            print("Switched to LAND mode.")

    def process_frame(self):
        """Process a frame, detect target, and send landing commands."""
        closest_target = self.detector.detect()
        if closest_target:
            x, y, z, x_ang, y_ang, payload, draw_img = closest_target
            if self.vehicle.location.global_relative_frame.alt > 0.1:
                self.switch_to_land_mode()
                self.send_land_message(x_ang, y_ang, float(z) / 100)
                if not self.simulation: # By testing, we found erros if not sent twice
                    self.send_land_message(x_ang, y_ang, float(z) / 100)
                print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')

            else:
                print("Finishing landing.")
                self.landed = True

    def main_loop(self):
        """Main loop for real-world execution."""
        while self.vehicle.armed and not self.landed:
            self.process_frame()

    def start(self):
        """Entry point to start the process."""
        if not self.landed:
                self.main_loop()
                self.do_plnd = True

    def reset(self):
        """Entry point to reset the process."""
        self.do_plnd = False
        self.landed = False
    