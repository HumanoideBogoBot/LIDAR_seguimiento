#!/usr/bin/env python 
#Leonardo Gracida Munoz A01379812
#Daniel Fuentes Castro A01750425
#Santiago Ortiz Suzarte A01750402
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

class RightHandRuleController:
    def __init__(self, wall_dist=0.5, w_max = 1, v_max=0.2,xt = 5,yt = 5):
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.wr_listener = rospy.Subscriber('/wr', Float32,self.wr_callback)
        self.wl_listener = rospy.Subscriber('/wl', Float32,self.wl_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(50.0)
        self.wall_dist = wall_dist
        self.w_max = w_max
        self.v_max = v_max
        self.scan = None
        self.xt = xt
        self.yt = yt
        self.x = 0
        self.y = 0
        self.th = 0
        self.wr = 0
        self.wl = 0
        self.r =  0.05
        self.l = 0.188
        self.distance_ahead = 0 
        self.distance_to_right = 0
        self.rayo_izq = 0
        self.rayo_der = 0
        self.eth = 0
        self.xin = 0
        self.yin = 0
        self.thin = 0
        self.alpha = 0
        

    def scan_callback(self, msg):
        self.scan = msg
    
    def wr_callback(self, msg):
        self.wr = msg.data
    def wl_callback(self, msg):
        self.wl = msg.data
        
    def follow_left_hand_wall(self):
        #print(len(self.scan.ranges))
        if self.scan is not None:
            #Obtenemos el angulo de inclinacion que tiene el robot conforme al muro                               
            alpha = find_wall_direction(self.scan)
            #print(alpha)
            #Obtenemos la distancia perpendicular que tiene el robot conforme al muro
            perpendicular_wall_dis = self.distance_to_left * np.cos(alpha)
            #print(distance_ahead)
            #Declaramos el angulo de inclinacion Deseado
            alpha_des = 0
            #Declaramos a que distancia deseamos estar alejados del muro
            perpendicular_wall_dis_des = self.wall_dist
            #Obtenemos el error de angulo de distancia hacia el muro
            error_alpha = alpha_des - alpha
            error_perpendicular_wall_dis = perpendicular_wall_dis_des - perpendicular_wall_dis
            #Definimos las constantes proporcionales del sistema de control
            kp_alpha = 1
            kp_perpendicular_wall_dis = 1
            #Sumamos lo de los dos controladores proporcionales para obtener la velocidad angular
            w = (kp_alpha * error_alpha)# + (kp_perpendicular_wall_dis * error_perpendicular_wall_dis)
            #--------------------------------------------------------------
            #--------------------------------------------------------------
            #Definimos el limte del filtro de saturacion
            limite = 1.5
            #Defimimos la distancia maxim que nos podemos hacercar de frente con el robot
            limite_dis_ahead = 1
            #Si estamos muy cerca del muro giramosa la izquierda.
            #Creamos el filtro de saturacion.
            v = self.v_max
            if w > self.w_max:
                w = self.w_max
            elif w < -1*self.w_max:
                w = -1*self.w_max
            #print(distance_ahead)
            if self.distance_ahead < limite_dis_ahead:
                w = 1
                v = 0
            if (self.distance_to_left > 2) and ((self.distance_ahead or self.rayo_izq or self.rayo_der) < limite_dis_ahead):
                w = 1
            
            return (v,-w)
        else:
            return (0.0,0.0)
            
    def follow_right_hand_wall(self):
        #print(len(self.scan.ranges))
        if self.scan is not None:
            #Obtenemos el angulo de inclinacion que tiene el robot conforme al muro                               
            alpha = find_wall_direction_right(self.scan)
            #print(alpha)
            #Obtenemos la distancia perpendicular que tiene el robot conforme al muro
            perpendicular_wall_dis = self.distance_to_right * np.cos(alpha)
            #print(distance_ahead)
            #Declaramos el angulo de inclinacion Deseado
            alpha_des = 0
            #Declaramos a que distancia deseamos estar alejados del muro
            perpendicular_wall_dis_des = self.wall_dist
            #Obtenemos el error de angulo de distancia hacia el muro
            error_alpha = alpha_des - alpha
            error_perpendicular_wall_dis = perpendicular_wall_dis_des - perpendicular_wall_dis
            #Definimos las constantes proporcionales del sistema de control
            kp_alpha = 1
            kp_perpendicular_wall_dis = 1
            #Sumamos lo de los dos controladores proporcionales para obtener la velocidad angular
            w = (kp_alpha * error_alpha) + (kp_perpendicular_wall_dis * error_perpendicular_wall_dis)
            #--------------------------------------------------------------
            #--------------------------------------------------------------
            #Definimos el limte del filtro de saturacion
            limite = 1.5
            #Defimimos la distancia maxim que nos podemos hacercar de frente con el robot
            limite_dis_ahead = 0.5
            #Si estamos muy cerca del muro giramosa la izquierda.
            #Creamos el filtro de saturacion.
            if w > self.w_max:
                w = self.w_max
            elif w < -1*self.w_max:
                w = -1*self.w_max
            #print(distance_ahead)
            if (self.distance_ahead or self.rayo_izq or self.rayo_der) < limite_dis_ahead:
                w = 1
            
            return (self.v_max,w)
        else:
            return (0.0,0.0)
    
    def cal_odometry(self,x,y,th,dt):
        x = x + self.r*((self.wr+self.wl)/(2))*dt*np.cos(th)
        y = y + self.r*((self.wr+self.wl)/(2))*dt*np.sin(th)
        th = th + self.r*((self.wr-self.wl)/(self.l))*dt
        return (x,y,th)

    def odometry(self,dt,xin,yin):
        #ther = np.arctan2(self.xt,self.yt) - 
        #t0 = rospy.get_rostime().to_sec()
        x,y,th = self.cal_odometry(self.x,self.y,self.th,dt)
        if th > np.pi:
            th = th - 2*np.pi
        if th < -np.pi:
            th = th + 2*np.pi
        self.x = x
        self.y = y
        self.th = th
        thi = np.arctan2(self.yt-yin,self.xt-xin)
        if thi > np.pi:
            thi = th - 2*np.pi
        if thi < -np.pi:
            thi = thi + 2*np.pi
        #thr = np.arctan2(self.x,self.y)    
        ther = thi - self.th
        self.eth = ther
        edis = np.sqrt((self.xt - self.x)**2 + (self.yt - self.y)**2)
        #print(edis)
        kd,kth = 1,1
        if ther > np.pi:
            ther = th - 2*np.pi
        if ther < -np.pi:
            ther = thi + 2*np.pi
        w = (ther*kth)
        v = kd * edis
        if w > 2:
                w = 2
        elif w < -1*2:
            w = -1*2
        if v > self.v_max:
                v = self.v_max
        elif v < -1*self.v_max:
            v = -1*self.v_max
        #print(v,w)
        if edis < 0.4:
            v,w = 0,0
        print(edis)
        return (v,w)


        
    def main(self):
        t0 = rospy.get_rostime().to_sec()
        estado_odom = True
        estado_esquina = False

        while not rospy.is_shutdown():
            if self.scan is None:
                continue
            self.distance_ahead = (get_distance_in_sector(self.scan,np.pi,np.pi - 2*np.pi/180)+get_distance_in_sector(self.scan,-np.pi + 2*np.pi/180,-np.pi))/2
            
            self.distance_to_left = get_distance_in_sector(self.scan,
                                                    -np.pi/2 + 4*np.pi/180,
                                                    -np.pi/2)
            self.distance_to_right = get_distance_in_sector(self.scan,
                                                    np.pi/2,
                                                    np.pi/2 - 4*np.pi/180)
            self.rayo_izq = get_distance_in_sector(self.scan,
                                                    np.radians(-133),
                                                    np.radians(-137))
            self.rayo_der = get_distance_in_sector(self.scan,
                                                    np.radians(137),
                                                    np.radians(133))
            tf = rospy.get_rostime().to_sec()
            dt = tf- t0
            if dt > 0.1:
                dt = 0.0
            v,w = self.odometry(dt,self.xin,self.yin)
            #print(self.distance_to_right)
            if (estado_odom ==  True) and ((self.distance_ahead < 1) or (self.rayo_izq < 0.75)or (self.rayo_der < 0.75)):
                estado_odom = False
                #estado_giro = True
            if (estado_odom == False) and ((self.distance_ahead > 1) and (self.rayo_izq > 1) and(np.abs(self.eth) < np.pi/4)):
                estado_esquina = True
                self.xin = self.x
                self.yin = self.y
            if (estado_odom == False) and (estado_esquina == True):
                w = 0 #-(self.wall_dist - self.distance_to_left)
                v = self.v_max
                if (self.distance_to_left > 0.5):
                    estado_odom = True
                    estado_esquina = False
            print(self.distance_ahead,self.rayo_izq,self.rayo_der)
            if (estado_odom == False) and (estado_esquina == False):
                print("wall follow")
                v,w = self.follow_left_hand_wall()
            else:
                print("odom")
            

            msg = Twist()
            msg.angular.z = w
            msg.linear.x = v
            self.vel_pub.publish(msg)
            #print("estado odom",self.xin,self.yin,self.thin)
            t0 = tf
            #print(perpendicular_wall_dis)
            self.rate.sleep()       

def range_index(scan, angle):
    #--------------------------------------------------------------
    # Your code here
    #Obtenemos que tan largo es el index del abanico del LIDAR
    num_scans = len(scan.ranges)
    #Obtenemos el angulo maximo y minimo del abanico
    maximo = scan.angle_max
    minimo = scan.angle_min
    #print(maximo)
    #Mapeamos del menor al mayor partiendo el abanico 720 partes
    abanico = np.linspace(minimo,maximo,num_scans)
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
    theta = -135
    #Obtenemos la distancia de la hipotenusa de nuestro triangulo
    a = get_distance_in_sector(scan, np.radians(theta+2), np.radians(theta-2))
    #Obtenemos la idstancia perpendicular del robot con el muro
    b = get_distance_in_sector(scan, np.radians(-88), np.radians(-92))
    #print(a,b)
    #Obtenemos el angulo de inclinacion del robot conforme al muro
    alpha = np.arctan2((-a*np.cos(np.radians(theta))-b),(-a*np.sin(np.radians(theta))))
    #print(a,b)
    #print(alpha)
    #print((-a*np.sin(np.radians(theta))))
    return alpha
    #--------------------------------------------------------------
    #--------------------------------------------------------------


def find_wall_direction_right(scan):
    #--------------------------------------------------------------
    # Your code here
    #Declaramos el angulo de inclinacion del rayo que va a ser la hipotenusa de nuestro triangulo
    theta = 135
    #Obtenemos la distancia de la hipotenusa de nuestro triangulo
    a = get_distance_in_sector(scan, np.radians(theta+2), np.radians(theta-2))
    #Obtenemos la idstancia perpendicular del robot con el muro
    b = get_distance_in_sector(scan, np.radians(92), np.radians(88))
    print(a,b)
    #Obtenemos el angulo de inclinacion del robot conforme al muro
    alpha = np.arctan2((-a*np.cos(np.radians(theta))-b),(a*np.sin(np.radians(theta))))
    #print(-a*np.cos(np.radians(theta))-b)
    print(np.degrees(alpha))
    #print((-a*np.sin(np.radians(theta))))
    return alpha
    
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
    return np.mean(scan.ranges[end_index:start_index])





if __name__ == '__main__':

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController(xt = 7, yt = 3)
    rhw.main()
    

