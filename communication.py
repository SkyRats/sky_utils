import rospy
import tf
from pymavlink import mavutil
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL, CommandBool
from mavros_msgs.msg import State
import math
import argparse
from dronekit import connect, VehicleMode
import numpy as np
import time
import tf.transformations
PI = np.pi
HALF_PI = PI/2.0
TOL = 0.5

DRONEKIT = False

class Mav:
    """
    Interface with mavros
    """

    def __init__(self, debug : bool = False, use_dk : bool = False) -> None:
        
        #INITIALIZING NODE
        rospy.init_node("mav")

        #USE DRONEKIT if REQUESTED

        if use_dk:
            parser = argparse.ArgumentParser()

            parser.add_argument('--connect', default='udp:127.0.0.1:14551')


            args = parser.parse_args()

            #-- Connect to the vehicle
            print('Connecting...')
            self.vehicle = connect(args.connect)

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

            # Ensure the vehicle is in GUIDED mode
            if self.vehicle.mode != 'GUIDED':
                self.vehicle.mode = VehicleMode('GUIDED')
                while self.vehicle.mode != 'GUIDED':
                    time.sleep(1)
                print('Vehicle in GUIDED mode')

        #SUBSCRIBERS
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        #PUBLISHERS
        self.pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        

        #SERVICES
        self.mode_serv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_serv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_serv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        #ATTRIBUTES
        self.pose = Pose()
        self.rate = rospy.Rate(60)
        self.goal_pose = Pose()
        self.debug = debug
        self.mode = int()
        self.prev_vy = 0
        self.prev_vx = 0
        self.prev_vz = 0
        self.drone_state = State()

    def pose_callback(self, msg : PoseStamped) -> None:
        """
        ROS callback used to get local position PoseStamped messages.
        """
        self.pose = msg.pose

    def set_vel(self, vel_x : float=0, vel_y : float=0, vel_z : float=0, ang_x : float=0, ang_y : float=0, ang_z : float=0) -> None:
        """
        Populates a Twist object with velocity information
        """
        twist = Twist()

        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = vel_z

        twist.angular.x = ang_x
        twist.angular.y = ang_y
        twist.angular.z = ang_z

        self.vel_pub.publish(twist)

    def state_callback(self, state_data):

        self.drone_state = state_data


    def publish_pose(self, pose : Pose) -> None:
        """
        Populates a PoseStamped object with pose and publishes it
        """
        stamped = PoseStamped()
        stamped.pose = pose

        self.pos_pub.publish(stamped)
    
    def move_drone_with_velocity_dk(self, vx, vy, vz, alpha):
        """
        Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
        location in the North, East, Down frame.
        """
        # Apply smoothing using a low-pass filter
        smoothed_vx = alpha * vx + (1 - alpha) * self.prev_vx
        smoothed_vz = alpha * vz + (1 - alpha) * self.prev_vz
        smoothed_vy = alpha * vy + (1 - alpha) * self.prev_vy
        
        print("Velocities X and Y: ", smoothed_vx,smoothed_vz)

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b110111000111, # type_mask (only positions enabled)
            0, 0, 0,
            smoothed_vx, smoothed_vy, smoothed_vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

        # Update previous velocities
        self.prev_vx = smoothed_vx
        self.prev_vz = smoothed_vz
        self.prev_vy = smoothed_vy
    
    def stop_dk(self, vx : bool = True, vy : bool = True, vz : bool = True):

        """
        Stopes movement in any axis

        """

        if vx:
            vx = 0
        else:
            vx = self.prev_vx
        
        if vy:
            vy = 0
        else:
            vy = self.prev_vy
        
        if vz:
            vz = 0

        else: 
            vz = self.prev_vz

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
            0b110111000111, # type_mask (only positions enabled)
            0, 0, 0,
            vx, vy, vz, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        # Send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
    
    def end_dk(self):

        # END CONNECTION

        self.vehicle.close()
    

    def takeoff_dk(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude - using dronekit
        """

        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def goto(self, x=None, y=None, z= None, yaw=None) -> None:
        """
        Sends a Pose message and publishes it as a setpoint (assuming vehicle is in guided mode). Yaw is offset by pi/2.
        If movement in a specifit axis is not provided, assumes that you want to keep the vehicles current axial position.
        Updates the self.goal_pose variable as well
        """

        #if new position on axis is provided use it, otherwise just keep current one
        self.goal_pose.position.x = x if x != None else self.pose.position.x
        self.goal_pose.position.y = y if y != None else self.pose.position.y
        self.goal_pose.position.z = z if z != None else self.pose.position.z

        #if new yaw was given, use it. Otherwise keep the vehicles current yaw
        if yaw != None:
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw + HALF_PI)
            self.goal_pose.orientation.x = quat[0]
            self.goal_pose.orientation.y = quat[1]
            self.goal_pose.orientation.z = quat[2]
            self.goal_pose.orientation.w = quat[3]
        else:
            self.goal_pose.orientation.x = self.pose.orientation.x
            self.goal_pose.orientation.y = self.pose.orientation.y
            self.goal_pose.orientation.z = self.pose.orientation.z
            self.goal_pose.orientation.w = self.pose.orientation.w

        if self.debug:
            rospy.loginfo(f"[GOTO] Sending goto {self.goal_pose}")

        self.publish_pose(pose=self.goal_pose)

    def wait_position(self, min_distance : float, wait_time : float=0.3) -> None:
        """
        Locks code execution while not close enough to goal position
        """
        if self.debug:
            rospy.loginfo(f"[WAIT_POSITION] Locking process until at least {min_distance} meters close")
            rospy.loginfo(f"[WAIT_POSITION] Current position\n{self.pose}")
            rospy.loginfo(f"[WAIT_POSITION] goal position\n{self.goal_pose}")

        distance = self.distance_to_goal()
        
        while(distance > min_distance):

            if self.debug:
                rospy.loginfo(f"[WAIT_POSITION] Still haven't arrived at position. Current distance is {distance} meters. Waiting for {wait_time} seconds")

            rospy.sleep(wait_time)

            distance = self.distance_to_goal()

        if self.debug:
            rospy.loginfo(f"[WAIT_POSITION] Arrived at {self.pose}. Unlocking process")

    def distance_to_goal(self) -> float:
        """
        Calculates the euclidian distance between current position and goal position
        """
        dx = self.goal_pose.position.x - self.pose.position.x
        dy = self.goal_pose.position.y - self.pose.position.y
        dz = self.goal_pose.position.z - self.pose.position.z

        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
    
    def wait_angle(self, min_angle : float, wait_time : float =0.3):
        """
        Locks code execution while not close enough to goal_angle
        """
        if self.debug:
            rospy.loginfo(f"[WAIT_ANGLE] Checking current and goal position and locking process until at least {min_angle} meters close")

        angle = self.angle_to_goal()
        
        while(angle > min_angle):

            if self.debug:
                rospy.loginfo("[WAIT_ANGLE] Still haven't turned to angle. Current angle is {angle} meters. Waiting for {wait_time} seconds")

            rospy.sleep(wait_time)

            angle = self.angle_to_goal()

        if self.debug:
            rospy.loginfo(f"[WAIT_ANGLE] Arrived at {self.pose.orientation}. Unlocking process")
        
    def angle_to_goal(self) -> float:
        """
        Calculates the difference between goal yaw and current yaw
        """
        yaw_current = tf.transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, 
                                                            self.pose.orientation.z, self.pose.orientation.w])[2]

        yaw_goal = tf.transformations.euler_from_quaternion([self.goal_pose.orientation.x, self.goal_pose.orientation.y, 
                                                            self.goal_pose.orientation.z, self.goal_pose.orientation.w])[2]
        
        return yaw_goal - yaw_current

    def rotate(self, yaw : float) -> None:
        """
        Rotates vehicles yaw. The same as a goto but keeping the current position
        """

        self.goto(x=self.pose.position.x, y=self.pose.position.y, z=self.pose.position.z, yaw=yaw)


    def arm(self) -> bool:
        """
        Arms throttle
        """
        return self.arm_serv(True)

    def disarm(self) -> bool:
        """
        Disarms throttle
        """
        return self.arm_serv(False)

    def takeoff(self, height : float= 1) -> bool:
        """
        Changes vehicle mode to guided, arms throttle and takes off
        """
        velocity = 1
        self.change_mode("4")
        success = False

        while not success:

            if not self.drone_state.armed:
                rospy.logwarn("Arming drone...")
                fb = self.arm()

                while not fb.success:

                    fb = self.arm()
                    self.rate.sleep()

                rospy.loginfo("Drone armed \n")

            else:
                rospy.loginfo("Drone already armed \n")
            
            rospy.logwarn("Executing takeoff...")

            # message = self.takeoff_serv(altitude = height)
            # success = message.success
            rospy.sleep(2)

            if not success:
                p = self.pose.position.z
                time=0
                while abs(self.pose.position.z - height) >= TOL and not rospy.is_shutdown():

                    time += 1/60.0 #sec - init_time
                    
                    rospy.logwarn('Taking off at ' + str(velocity) + ' m/s')   
                    
                    if p < height:
                        self.set_vel(0,0,1,0,0,0)
                        # p = ((-2 * (velocity**3) * (time**3)) / height**2) + ((3*(time**2) * (velocity**2))/height)
                        # self.goto(self.pose.position.x, self.pose.position.y, p)

                    else:
                        self.goto(self.pose.position.x, self.pose.position.y, height)
            success = True
                
            # success = self.takeoff_serv(altitude = height)


        # return True

        # return self.change_mode("4") and self.arm() and self.takeoff_serv(latitude = 0.0, longitude = 0.0, altitude = height)

    def change_mode(self, mode : str) -> bool:
        """
        Changes vehicle mode to given string. Some modes include:
        STABILIZE = 0,
        GUIDED = 4,
        LAND = 9
        """
        return self.mode_serv(custom_mode = mode)

    def land(self) -> bool:
        """
        Sets mode to land and disarms drone.
        """
        return self.change_mode("9") and self.disarm()
    

def test():
    mav = Mav(debug=True)

    rospy.loginfo("Taking off...")

    if not mav.takeoff(5):
        print("Couldn't takeoff.")
        return
    else:
        rospy.sleep(6)

    mav.goto(x=2)
    mav.wait_position(0.15)
    
    mav.rotate(np.pi)
    mav.wait_angle(np.pi/9)

    mav.goto(x=0, y=0, z=1, yaw=0)
    mav.wait_position(0.15)

    rospy.loginfo("Trying to land...")
    if not mav.land():
        rospy.loginfo("Couldnt't land.")

if __name__=="__main__":
    test()