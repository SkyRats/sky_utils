import rospy
import tf
from geometry_msgs.msg import Pose, Point, PoseStamped, Twist
from mavros_msgs.srv import SetMode, CommandTOL, CommandBool
from mavros_msgs.msg import PositionTarget
from simple_pid import PID
import math
import numpy as np
from time import time

import tf.transformations
PI = np.pi
HALF_PI = PI/2.0

class Mav:
    """
    Interface with mavros
    """

    def __init__(self, debug : bool = False, simulation: bool = False, lidar_min: float = 0.3) -> None:

        #INITIALIZING NODE
        rospy.init_node("mav")
        self.simulation = simulation
        self.lidar_min_distance = lidar_min

        if debug: rospy.loginfo("started mav")

        #SUBSCRIBERS
        if simulation: rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        else: rospy.Subscriber("/mavros/vision_pose/pose", PoseStamped, self.pose_callback)

        #PUBLISHERS
        self.pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.raw_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)


        #SERVICES
        self.mode_serv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_serv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_serv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        #ATTRIBUTES
        self.pose = Pose()
        self.goal_pose = Pose()
        self.debug = debug
        self.mode = int()

        #rospy.spin()
    
    def in_between(self, check, center, margin):
        """
        This function checks if a number is in certain interval of other number

        check: number to be compared
        center: reference number
        margin: maximum interval allowed
        """
        return (check <= center + margin and check >= center - margin)

    def pose_callback(self, msg : PoseStamped) -> None:
        """
        ROS callback used to get local position PoseStamped messages.
        """
        self.pose = msg.pose
        if not self.simulation: self.pose.position.z += self.lidar_min_distance

    def set_vel(self, vel_x : float=0.0, vel_y : float=0.0, vel_z : float=0.0, ang_x : float=0.0, ang_y : float=0.0, ang_z : float=0.0) -> None:
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

    def set_vel_relative(self, forward: float = 0.0, sideways: float = 0.0, upward:float = 0.0) -> None:

        res = PositionTarget()
        res.header.stamp = rospy.Time.now()
        res.header.frame_id = "base_footprint"

        res.velocity.x = forward
        res.velocity.y = sideways
        res.velocity.z = upward
        res.coordinate_frame = PositionTarget.FRAME_BODY_NED
        res.type_mask = PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.FORCE |PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

        self.raw_pub.publish(res)


    def publish_pose(self, pose : Pose) -> None:
        """
        Populates a PoseStamped object with pose and publishes it
        """
        stamped = PoseStamped()
        stamped.pose = pose

        self.pos_pub.publish(stamped)

    def goto(self, x=None, y=None, z=None, yaw=None, send_time=None) -> None:
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

        if self.debug: rospy.loginfo(f"[GOTO] Sending goto {self.goal_pose}")

        if send_time is not None:

            if self.debug: rospy.loginfo(f"[GOTO] Keep sending goto for {send_time} seconds")

            start_time = time()
            while time() - start_time < send_time:
                self.publish_pose(pose=self.goal_pose)
        else:
            self.publish_pose(pose=self.goal_pose)
        rospy.loginfo(f"[GOTO] Finished")


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

    def rotate_control_yaw(self, yaw : float, yaw_rate: float = 0.5) -> None:
        """
        Rotates vehicles yaw. Its using setpoint raw
        """
        angle = PositionTarget()
        angle.header.stamp = rospy.Time.now()
        angle.header.frame_id = "base_footprint"

        angle.yaw = yaw
        angle.yaw_rate = yaw_rate

        angle.coordinate_frame = PositionTarget.FRAME_BODY_NED
        angle.position.x = self.pose.position.x
        angle.position.y = self.pose.position.y
        angle.position.z = self.pose.position.z
        angle.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ

        self.raw_pub.publish(angle)

        if self.debug: rospy.loginfo(f"[ROTATE] Rotating to {yaw} rad, with a rate of {yaw_rate} rad/2")

    def rotate(self, yaw : float) -> None:
        """
        Rotates vehicles yaw. The same as goto but only changes yaw
        """
        self.goto(x=self.pose.position.x, y=self.pose.position.y, z=self.pose.position.z, yaw=yaw)


    def rotate_relative(self, yaw : float) -> None:
        """
        Rotates vehicles yaw in relative of its current position. Using the rotate method
        """

        yaw_current = tf.transformations.euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y,
                                                            self.pose.orientation.z, self.pose.orientation.w])[2]

        self.rotate(yaw=(yaw_current + yaw - HALF_PI))

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

        if self.in_between(self.pose.position.z, height, 0.1): return True

        prev_pose = self.pose

        res = self.change_mode("4") and self.arm() and self.takeoff_serv(altitude = height)
        rospy.sleep(3)
        self.goto(x = prev_pose.position.x, y = prev_pose.position.y, z = height, yaw=0, send_time=5)
        rospy.sleep(3)

        if self.pose.position.z < height: return False
        return res

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

class TwoAxisPID:
    """
    Simply put, this is a PID in two axis. Used in centralizing tasks.
    """

    def __init__(self, Kp_x : float, Ki_x : float, Kd_x : float, Kp_y : float, Ki_y : float, Kd_y : float, setpoint_x: float, setpoint_y: float) -> None:
        self.pid_x = PID(Kp_x, Ki_x, Kd_x, setpoint = setpoint_x)
        self.pid_y = PID(Kp_y, Ki_y, Kd_y, setpoint = setpoint_y)

    def update(self, error_x: float, error_y : float) -> tuple:
        """
        Updates both PID controllers with an error considering a delta time since the start of the iteration
        """
        return (self.pid_x(error_x), self.pid_y(error_y))

    def refesh(self) -> None:
        """
        Resets the PID controllers
        """
        self.pid_x.reset()
        self.pid_y.reset()
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
