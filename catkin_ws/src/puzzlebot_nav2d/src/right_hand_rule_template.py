#!/usr/bin/env python 
#Leonardo Gracida Munoz A01379812
#Daniel Fuentes Castro A01750425
#Santiago Ortiz Suzarte A01750402
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RightHandRuleController:
    def __init__(self, wall_dist=0.5, w_max = np.pi, v_max=0.4 ):
        """
        Arguments
        --------------
        wall_dist  : float
           Desired distance to wall
        w_max  : float
           Max angular velocity
        v_max  : float
           Max linear velocity
        """
        self.scan_listener = rospy.Subscriber('/laser/scan', LaserScan,
                                              self.scan_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(10.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None

    def scan_callback(self, msg):
        """Called when a new scan is available from the lidar. """
        self.scan = msg

    def go_to_wall(self):
        """ Go straight ahead until at desired distance from a wall. """
        #--------------------------------------------------------------
        # Your code here
        #--------------------------------------------------------------
        #--------------------------------------------------------------

        pass
        
    def follow_right_hand_wall(self):
        found_wall = False
        while not rospy.is_shutdown():

            if self.scan is not None:
                #--------------------------------------------------------------
                # Your code here
                # 1) Check distance to wall/obstacle in front and to the right.
                # 2) Based on the information in 1) calculate a suitable angular
                #    velocity, w.
                # 3) Publish the Twist message to steer the puzzlebot.

                distance_ahead = get_distance_in_sector(self.scan,
                                                        0 - 2*np.pi/180,
                                                        0 + 2*np.pi/180)
                distance_to_right = get_distance_in_sector(self.scan,
                                                           np.pi/2 - 4*np.pi/180,
                                                           np.pi/2)
                #Obtenemos el angulo de inclinacion que tiene el robot conforme al muro                               
                alpha = find_wall_direction(self.scan)
                #Obtenemos la distancia perpendicular que tiene el robot conforme al muro
                perpendicular_wall_dis = distance_to_right * np.cos(alpha)
                #Declaramos el angulo de inclinacion Deseado
                alpha_des = 0
                #Declaramos a que distancia deseamos estar alejados del muro
                perpendicular_wall_dis_des = 0.5
                #Obtenemos el error de angulo de distancia hacia el muro
                error_alpha = alpha_des - alpha
                error_perpendicular_wall_dis = perpendicular_wall_dis_des - perpendicular_wall_dis
                #Definimos las constantes proporcionales del sistema de control
                kp_alpha = 1
                kp_perpendicular_wall_dis = 0.75
                #Sumamos lo de los dos controladores proporcionales para obtener la velocidad angular
                w = (kp_alpha * error_alpha) + (kp_perpendicular_wall_dis * error_perpendicular_wall_dis)
                #--------------------------------------------------------------
                #--------------------------------------------------------------
                #Definimos el limte del filtro de saturacion
                limite = 1.5
                #Defimimos la distancia maxim que nos podemos hacercar de frente con el robot
                limite_dis_ahead = 1
                #Si estamos muy cerca del muro giramosa la izquierda.
                if distance_ahead <= limite_dis_ahead:
                    w = 100
                #Creamos el filtro de saturacion.
                if w > limite:
                    w = limite
                elif w < -1*limite:
                    w = -1*limite
                
                msg = Twist()
                msg.angular.z = w
                msg.linear.x = 0.5
                self.vel_pub.publish(msg)
                
            self.rate.sleep()    

def range_index(scan, angle):
    #--------------------------------------------------------------
    # Your code here
    #Obtenemos que tan largo es el index del abanico del LIDAR
    num_scans = len(scan.ranges)
    #Obtenemos el angulo maximo y minimo del abanico
    maximo = scan.angle_max
    minimo = scan.angle_min
    #Mapeamos del menor al mayor partiendo el abanico 720 partes
    abanico = np.linspace(maximo,minimo,num_scans)
    #Obtenemos distancia euclidiana del angulo deseado con todo el abanico
    dist = np.sqrt((abanico-angle)**2)
    #obtenemos el index con el que se tuvo una distancia menor para obtener el index del abanico correspondiente
    index = np.argmin(dist)
    return index
    #--------------------------------------------------------------
    #--------------------------------------------------------------    
            


def find_wall_direction(scan):
    #--------------------------------------------------------------
    # Your code here
    #Declaramos el angulo de inclinacion del rayo que va a ser la hipotenusa de nuestro triangulo
    theta = 45
    #Obtenemos la distancia de la hipotenusa de nuestro triangulo
    a = get_distance_in_sector(scan, np.radians(theta-2), np.radians(theta+2))
    #Obtenemos la idstancia perpendicular del robot con el muro
    b = get_distance_in_sector(scan, np.radians(84), np.radians(90))
    #Obtenemos el angulo de inclinacion del robot conforme al muro
    alpha = np.arctan2((a*np.cos(np.radians(theta))-b),(a*np.sin(np.radians(theta))))
    return alpha
    #--------------------------------------------------------------
    #--------------------------------------------------------------

    
def get_distance_in_sector(scan, start_angle, end_angle) :
    num_scans = len(scan.ranges)
    #--------------------------------------------------------------
    # Your code here. For documentation of the LaserScan message:
    # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
    #
    # 1) Find the indices into scan.ranges corresponding to the start_ange and end_angle
    # 2) Compute the average range in the sector using np.mean()
    #--------------------------------------------------------------
    #Obtenemos el index del angulo incial
    start_index = range_index(scan,start_angle)
    #Obtenemos el index de angulo final
    end_index = range_index(scan,end_angle)
    #Como en este caso el ultmo index numpy no lo cuenta, en caso de en final estar en el sector negativo lo recorremos para obtener el ultimo valor
    if (start_angle < 0):
        start_index += 1
        end_index += 1
    return np.mean(scan.ranges[end_index:start_index])




def generate_test_scan(straight_wall=False):
    """Function used for testing. Will create and return a LaserScan message"""
    scan = LaserScan()
    scan.angle_min = -np.pi/2
    scan.angle_max = np.pi/2
    num_scans = 720 # 4 lines per degree
    scan.angle_increment = np.pi/num_scans
    scan.range_min = 0.1
    scan.range_max = 30

    scan.ranges = np.arange(0, 720.0) / 10.0
    if straight_wall: # The wall is on the right at a distance of 10m
        scan.ranges[:400] = scan.range_max
        dth = np.arange(0, 320)*scan.angle_increment
        for i in range(320):
            scan.ranges[719-i] = 10/np.cos(dth[i])


    return scan

if __name__ == '__main__':
    if len(sys.argv) > 1:
        if sys.argv[1] == "--test":
            import doctest
            doctest.testmod()
            sys.exit(0)

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController()
    rhw.follow_right_hand_wall()
    


