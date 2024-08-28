#!/usr/bin/env python

import rospy
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


def quaternions_to_euler_angle(w, x, y, z):
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


def convert_body_to_local(body_point, drone_position, drone_ori):
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


def go_to_local(pub, goal_x, goal_y, goal_z, yaw = None, sleep_time=5):
    rospy.loginfo("Going towards local position: (" + str(goal_x) + ", " + str(goal_y) + ", " + str(goal_z) + "), with a yaw angle of: " + str(yaw))

    init_time = now = time.time()
    while now-init_time < sleep_time:
        set_position(pub, goal_x, goal_y, goal_z, yaw)
        now = time.time()
    
    # rospy.loginfo("Arrived at requested position")
    
def set_position(setpoint_pub, x, y, z, yaw = None):

    goal_pose = PoseStamped()

    goal_pose.pose.position.x = float(x)
    goal_pose.pose.position.y = float(y)
    goal_pose.pose.position.z = float(z)
    [goal_pose.pose.orientation.x, 
    goal_pose.pose.orientation.y, 
    goal_pose.pose.orientation.z,
    goal_pose.pose.orientation.w] = quaternion_from_euler(0,0,yaw) #roll,pitch,yaw

    setpoint_pub.publish(goal_pose)


# Essa classe contém um subscriber no tópico de posição do aruco (sky_vision)
# Seu callback atualiza a posição do marcador nas coordenadas do drone
class ArucoSubscriber:
    def __init__(self):
        self.arucoPose = Vector3()
        self.aruco_count = 0
        rospy.Subscriber('/sky_vision/down_cam/aruco/pose', Point, self.aruco_pose_callback)
    
    def aruco_pose_callback(self, msg):
        try:
            self.arucoPose = [-msg.y/100, -msg.x/100, -msg.z/100]
            self.aruco_count += 1
        except:
            print("Erro ao pegar a posição do Aruco")
            self.arucoPose = None

    def get_pose(self):
        return np.array(self.arucoPose)
    
    def get_number(self):
        return self.aruco_count
    

# Essa classe atualiza a posição do drone nas coordenadas locais
# A partir dela, é possível consultar a posição atual do drone em x,y,z,roll,pitch,yaw
class PoseSubscriber:
    def __init__(self):
        self.dronePos = None
        self.droneOri = None
        self.aruco_count = 0
        rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.dronePosCallback)
    
    def dronePosCallback(self, msg):
        try:
            self.dronePos = msg.pose.position
            self.droneOri = msg.pose.orientation
            self.droneOri = quaternions_to_euler_angle(self.droneOri.w, self.droneOri.x, self.droneOri.y, self.droneOri.z)
        except:
            print("ERRO ao buscar posição")

    def get_drone_pose(self):
        return np.array([self.dronePos.x, self.dronePos.y, self.dronePos.z])
    
    def get_drone_ori(self):
        return np.array(self.droneOri)
    

# Essa classe permite enviar requisições para o sky_vision procurar arucos
class TypePublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/sky_vision/down_cam/type', String, queue_size=1)
        self.msg = String()

    def publish_type(self):
        self.msg.data = "aruco"
        try:
            self.pub.publish(self.msg)
        except:
            print("Publish error!")


### Missão ###
# Ir reto até encontrar um aruco
# Calcular a posição do marcador
# Centralizar nele

#----- MAVROS

rospy.init_node('type_publisher', anonymous=False)

vc = TypePublisher()

vc2 = ArucoSubscriber()
drone = PoseSubscriber()

STATE = 0  # Estado inicial
SPEED = 0.15  # velocidade frontal do drone (m/s)
REFINE = 3 # número de aproximações

# Setpoint raw
setpoint_ = PositionTarget()
vel_setpoint_ = Vector3()
setpoint_.coordinate_frame = PositionTarget.FRAME_BODY_NED

# Publisher para enviar comandos de velocidade
setpoint_pub_raw = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

# Publisher para enviar comandos de posição
setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=1)

#-----

time.sleep(3)

# Posição atual do drone
INITIAL_HEIGHT = drone.get_drone_pose()[2]

# Altura final desejada para centralizar
FINAL_HEIGHT = 0.7

# Variáveis úteis
ARUCO_FOUND = False
DRONE_STOPED = False
speed_count = 0

print("Iniciando procura por aruco...")

while not rospy.is_shutdown():

    # Envia uma requisição para o sky_vision
    vc.publish_type()

    # Drone está procurando um aruco
    if STATE == 0:
        
        # Se encontrar o aruco 4 vezes (para ser mais robusto)
        if vc2.get_number() == 4 or ARUCO_FOUND:
            
            # Se for a primeira vez que define o marcador
            if not ARUCO_FOUND:
                print("Aruco encontrado! Parando...")
                ARUCO_FOUND = True
                
            # Atribui velocidade zero
            vel_setpoint_.x = 0
            
            # Se tiver dado o comando de zero velocidade 100 vezes (para garantir que parou)
            if speed_count > 100:
                DRONE_STOPED = True
                speed_count = 0
            else:
                speed_count += 1
            
            # Quando o drone estiver parado, muda o estado
            if DRONE_STOPED:
                print("Drone estável, calculando posição do Aruco...")
                STATE = 1
        
        else:
            # Se ainda não tiver encontrado o Aruco, velocidade frontal padrão
            vel_setpoint_.x = SPEED

        # VELOCIDADE NO EIXO X (em simulação)
        setpoint_.velocity.x = vel_setpoint_.x
        setpoint_.velocity.y = 0
        setpoint_.velocity.z = 0

        # Envia a velocidade para a mavros
        setpoint_pub_raw.publish(setpoint_)

    # Aruco foi encontrado
    elif STATE == 1:

        for i in range(REFINE):

            print(f"\n{i+1}° APROXIMAÇÃO...")

            time.sleep(3)

            # Pega a posição e orientação atual do drone no local NED frame
            drone_pose = drone.get_drone_pose()
            drone_ori = drone.get_drone_ori()

            # Pega a posição do Aruco no Body NED Frame
            aruco_pose = vc2.get_pose()

            print()
            print(f"Orientação do Drone:                 x: {round(drone_ori[0], 2)} | y: {round(drone_ori[1], 2)} | z: {round(drone_ori[2], 2)}")
            print(f"Posição do Aruco no Body NED frame:  x: {round(aruco_pose[0], 2)} | y: {round(aruco_pose[1], 2)} | z: {round(aruco_pose[2], 2)}")
            print(f"Posição do Drone:                    x: {round(drone_pose[0], 2)} | y: {round(drone_pose[1], 2)} | z: {round(drone_pose[2], 2)}")

            # Converte a posição do Aruco para o local NED frame
            x_pos, y_pos, z_pos = convert_body_to_local(aruco_pose, drone_pose, drone_ori) #  drone_position, roll, pitch, yaw)
            print(f"Posição do Aruco no Local NED frame: x: {round(x_pos, 2)} | y: {round(y_pos, 2)} | z: {round(z_pos, 2)}")
            print()

            # Calcula a altura para descer, a última altura deve ser a final
            height = INITIAL_HEIGHT - (INITIAL_HEIGHT - FINAL_HEIGHT)/3 * (i+1)

            # Comando de posição
            go_to_local(setpoint_pub, goal_x=x_pos, goal_y=y_pos, goal_z=height, yaw=0, sleep_time=2)

            print(f"{i+1}° APROXIMAÇÃO finalizada!\n")
        
        # Finaliza
        STATE = 3

    elif STATE == 3:
        break

cv2.destroyAllWindows()
