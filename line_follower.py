#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, TwistStamped, Vector3, PoseStamped
from std_msgs.msg import Float64, Bool, String
from mavros_msgs.msg import PositionTarget



class LineFollower:
    def __init__(self, debug=False, desired_drone_position=0.0, desired_angle = 0.0, default_velocity=0.25) -> None:    
        #attributes
        self.debug = debug
        self.y_error = Float64()
        self.angle_error = Float64()
        self.line_detected = Bool()
        self.desired_drone_position = Float64()
        self.desired_angle = Float64()
        self.setpoint = PositionTarget()
        self.velocity_setpoint = Vector3()
        self.default_velocity = default_velocity
        self.current_yaw = 0.0
        
        self.desired_drone_position.data = desired_drone_position
        self.desired_angle.data = desired_angle 


        # ROS node
        rospy.init_node('line_follower')
        if self.debug:
            rospy.loginfo("Line follower node started")
        
        # Subscribers
        rospy.Subscriber('/sky_vision/down_cam/line/pose', Point, self.line_pose_callback)
        rospy.Subscriber('/drone/control_effort', Float64, self.drone_ajustment_callback)
        rospy.Subscriber('/angle/control_effort', Float64, self.angle_ajustment_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.drone_pose_callback)

        if self.debug:
            rospy.loginfo("Subscribers initialized")

        # Publishers
        self.setpoint_drone_pub = rospy.Publisher('/drone/setpoint', Float64, queue_size=10)
        self.current_drone_pub = rospy.Publisher('/drone/state', Float64, queue_size=10)
        self.setpoint_angle_pub = rospy.Publisher('/angle/setpoint', Float64, queue_size=10)
        self.current_angle_pub = rospy.Publisher('/angle/state', Float64, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.type_pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1)

        if self.debug:
            rospy.loginfo("Publishers initialized")

        rospy.spin()

    def drone_pose_callback(self, msg) -> None:
        _,_,self.current_yaw = tf.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.debug:
            rospy.logwarn(f"yaw: {self.current_yaw}")
    
    def line_pose_callback(self, msg) -> None:
        self.drone_error = msg.x
        self.angle_error = msg.y
        self.line_detected = msg.z
        if self.debug:
            rospy.loginfo(f"Line detected: {self.line_detected}, Y error: {self.y_error}, Angle error: {self.angle_error}")

        # send setpoint
        self.setpoint_drone_pub.publish(self.desired_drone_position)
        self.setpoint_angle_pub.publish(self.desired_angle)

        # send current state
        self.current_drone_pub.publish(self.drone_error)
        self.current_angle_pub.publish(self.angle_error)

    def drone_ajustment_callback(self, msg) -> None:
        if self.debug:
            rospy.loginfo("Drone ajustment callback")
        self.velocity_setpoint.x = 0
        self.velocity_setpoint.y = 0
        self.velocity_setpoint.z = 0

        self.setpoint.velocity = self.velocity_setpoint

    
    def angle_ajustment_callback(self, msg) -> None:
        if self.debug:
            rospy.loginfo("Angle ajustment callback")
        self.setpoint.yaw =  (msg.data * np.pi / 180) - self.current_yaw if abs(msg.data) < 5 else 0 #angle in radians
        #self.setpoint.yaw_rate = 0.0 # 0.5 rad/s
    
        self.publish_setpoint()

    def publish_setpoint(self) -> None:
        self.setpoint.header.stamp = rospy.Time.now()
        self.setpoint.header.frame_id = "base_footprint"
        self.setpoint.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | (PositionTarget.IGNORE_YAW if not self.setpoint.yaw else 1) | PositionTarget.IGNORE_YAW_RATE
        self.setpoint.coordinate_frame = PositionTarget.FRAME_BODY_NED

        if self.debug:
            rospy.loginfo("Publishing setpoint")
            rospy.loginfo(f"velocity: x: {self.setpoint.velocity.x}, y ={self.setpoint.velocity.y} | yaw: {self.setpoint.yaw * 180 / np.pi} degrees")
        self.setpoint_pub.publish(self.setpoint)

def main():
    LineFollower(debug=True)


if __name__ == '__main__':
    main()