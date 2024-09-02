#!/usr/bin/env python3

import rospy
from .communication import Mav
import cv2
from std_msgs.msg import String
import time
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Vector3
from std_msgs.msg import String
from mavros_msgs.msg import PositionTarget
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class Centralize:
    def __init__ (self, mav: Mav, simulation: bool = True) -> None:
        self.mav = mav
        self.debug = mav.debug
        self.arucoPose = Vector3()
        self.aruco_count = 0
        self.aruco_found = False
        self.simulation = simulation   
        self.state = "CENTRALIZE" # "FIND_ARUCO", "CENTRALIZE", "FINISHED"

        # Subscribers
        rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.aruco_pose_callback)

        # Publishers
        self.pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1)

    def quaternions_to_euler_angle(self, w, x, y, z) -> list:
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.degrees(math.atan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.degrees(math.asin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.degrees(math.atan2(t3, t4))

        return [X, Y, Z]

    def convert_body_to_local(self, body_point, drone_position, drone_ori):
        """
        Converte uma posição de um ponto no frame Body NED para o frame Local NED.
        
        Parameters:
        body_point (numpy.array): Posição do ponto no frame Body NED [x, y, z].
        drone_position (numpy.array): Posição do drone no frame Local NED [x, y, z].
        drone_ori (numpy.array): Orientação do drone no frame Local NED [roll, pitch, yaw].

        Returns:
        numpy.array: Posição do ponto no frame Local NED.
        """

        roll, pitch, yaw = drone_ori
        
        # Cria a matriz de rotação a partir dos ângulos de Euler (rolagem, arfagem, guinada)
        rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        
        # Converte o ponto do frame Body NED para o frame Local NED aplicando a rotação
        local_point = rotation.apply(body_point)
        
        # Adiciona a posição do drone no frame Local NED para obter a posição global
        local_point += drone_position
        
        return local_point
    
    def aruco_pose_callback(self, msg) -> None:
        try:
            self.arucoPose = [-msg.y/100, -msg.x/100, -msg.z/100]
            self.aruco_count += 1
        except:
            rospy.loginfo("Erro ao pegar a posição do Aruco")
            self.arucoPose = None

    def run (self, final_height: int, refine: int, speed: float = 0.25) -> bool:
        initial_height = self.mav.pose.position.z

        if self.state == "CENTRALIZE":
            for i in range(refine):
                rospy.loginfo(f"\n{i+1}° APROXIMAÇÃO...")

                rospy.sleep(3)

                # Pega a posição e orientação atual do drone no local NED frame
                drone_pose = np.array([self.mav.pose.position.x, self.mav.pose.position.y, self.mav.pose.position.z])
                drone_ori = np.array(self.quaternions_to_euler_angle(self.mav.pose.orientation.w, self.mav.pose.orientation.x, self.mav.pose.orientation.y, self.mav.pose.orientation.z))

                # Pega a posição do Aruco no Body NED Frame
                aruco_pose = np.array(self.arucoPose) 

                rospy.loginfo(f"Orientação do Drone:                 x: {round(drone_ori[0], 2)} | y: {round(drone_ori[1], 2)} | z: {round(drone_ori[2], 2)}")
                rospy.loginfo(f"Posição do Aruco no Body NED frame:  x: {round(aruco_pose[0], 2)} | y: {round(aruco_pose[1], 2)} | z: {round(aruco_pose[2], 2)}")
                rospy.loginfo(f"Posição do Drone:                    x: {round(drone_pose[0], 2)} | y: {round(drone_pose[1], 2)} | z: {round(drone_pose[2], 2)}")

                # Converte a posição do Aruco para o local NED frame
                x_pos, y_pos, z_pos = self.convert_body_to_local(aruco_pose, drone_pose, drone_ori) #  drone_position, roll, pitch, yaw)
                rospy.loginfo(f"Posição do Aruco no Local NED frame: x: {round(x_pos, 2)} | y: {round(y_pos, 2)} | z: {round(z_pos, 2)}")

                # Calcula a altura para descer, a última altura deve ser a final
                height = initial_height - (initial_height - final_height)/refine * (i+1)

                # Comando de posição
                self.mav.goto(x_pos, y_pos, height)
                rospy.sleep(6)

                rospy.loginfo(f"{i+1}° APROXIMAÇÃO finalizada!")
            return True
        return False
