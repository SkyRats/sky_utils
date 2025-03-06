#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tf
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget



class LineFollower:
    def __init__(self, debug=False, default_velocity=0.25, kp_angle=0.5, kp_x = 0.5, kd_x = 0.25, ki_x = 0.1) -> None:    
        #attributes
        self.debug = debug
        self.default_velocity = default_velocity
        self.current_yaw = 0.0

        # PID constants
        self.kp_drone = kp_x
        self.kd_drone = kd_x
        self.ki_drone = ki_x
        self.kp_angle = kp_angle
        self.last_drone_error = 0.0
        self.sum_drone_error = 0.0


        # ROS node
        rospy.init_node('line_follower')
        if self.debug:
            rospy.loginfo("Line follower node started")
        
        # Subscribers
        rospy.Subscriber('/sky_vision/down_cam/line/pose', Point, self.line_pose_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

        if self.debug:
            rospy.loginfo("Subscribers initialized")

        # Publishers
        self.setpoint_yaw_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.setpoint_velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.type_pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=10)

        if self.debug:
            rospy.loginfo("Publishers initialized")

    def deg2rad(self, deg: float) -> float:
        return deg*np.pi/180
        
    def drone_pose_callback(self, msg) -> None:
        _,_,self.current_yaw = tf.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    
    def line_pose_callback(self, msg) -> None:
        drone_error = msg.x
        angle_error = self.deg2rad(msg.y)    
        line_detected = msg.z

        if line_detected: 
            angle_error = abs(angle_error) - np.pi/2 if angle_error < 0 else np.pi/2 - angle_error  
            self.publish_setpoint(angle_error, drone_error)
 
    def publish_setpoint(self, yaw_error: float, drone_error: float) -> None:
        msg = Twist() # uses local frame
        msg.linear.x= np.round((self.kp_drone * drone_error) + ((drone_error - self.last_drone_error)*self.kd_drone) + (self.sum_drone_error * self.ki_drone), 3)
        msg.linear.y = self.default_velocity
        msg.angular.z = np.round(self.kp_angle * yaw_error, 3)

        self.last_drone_error = drone_error
        self.sum_drone_error += drone_error

        self.setpoint_velocity_pub.publish(msg) 
        rospy.sleep(0.1)

    def start_following(self, val) -> None:
        msg = String()
        msg.data = "line" if val else " "   
        self.type_pub.publish(msg) 
        

def main():
    follower = LineFollower(debug=True, default_velocity=0.1, kp_angle=0.6, kp_x=0.4, kd_x=0.1, ki_x=0.0)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown(): 
        try:
            follower.start_following(True)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    main()