import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL, CommandBool
import math
import numpy as np

import tf.transformations
PI = np.pi
HALF_PI = PI/2.0

class Mav:
    """
    Interface with mavros
    """

    def __init__(self, debug : bool = False) -> None:
        
        #INITIALIZING NODE
        rospy.init_node("mav")

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
        self.goal_pose = Pose()
        self.debug = debug
        self.mode = int()

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

    def publish_pose(self, pose : Pose) -> None:
        """
        Populates a PoseStamped object with pose and publishes it
        """
        stamped = PoseStamped()
        stamped.pose = pose

        self.pos_pub.publish(stamped)

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

        return self.change_mode("4") and self.arm() and self.takeoff_serv(altitude = height)

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