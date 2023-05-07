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
    def __init__(self, wall_dist=0.25, w_max = 0.6, v_max=0.2,xt = 5,yt = 5):
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.wr_listener = rospy.Subscriber('/wr', Float32,self.wr_callback)
        self.wl_listener = rospy.Subscriber('/wl', Float32,self.wl_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel' , Twist, queue_size=1 )
        self.rate = rospy.Rate(100.0)
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
            #Obtenemos la distancia perpendicular que tiene el robot conforme al muro
            perpendicular_wall_dis = self.distance_to_left * np.cos(alpha)
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
            limite_dis_ahead = 0.75
            #Creamos el filtro de saturacion.
            v = self.v_max
            if w > self.w_max:
                w = self.w_max
            elif w < -1*self.w_max:
                w = -1*self.w_max
            #Si estamos muy cerca del muro giramosa la izquierda.
            if self.distance_ahead < limite_dis_ahead:
                v=0
                w = 1
            #En caso de estar enfrente de algo o con algo cerca a la izqueirda o derecha girar a la izqueirda sin avanzar
            if ((self.distance_ahead < 0.5) or (self.rayo_izq < 0.3) or (self.rayo_der < 0.3)) and (self.distance_to_left > 2):
                v=0
                w = 1
            
            return (v,-w)
        else:
            return (0.0,0.0)
            
    
    def cal_odometry(self,x,y,th,dt):
        #Funcion que hace los calculos de odometria de la nueva posicion
        x = x + self.r*((self.wr+self.wl)/(2))*dt*np.cos(th)
        y = y + self.r*((self.wr+self.wl)/(2))*dt*np.sin(th)
        th = th + self.r*((self.wr-self.wl)/(self.l))*dt
        return (x,y,th)

    def odometry(self,dt,xin,yin):
        #Hacemos los nuevos calculos de la odometria
        x,y,th = self.cal_odometry(self.x,self.y,self.th,dt)
        #Hacemos que el calculo de inclinacion siempre vaya de 0 a -pi o de 0 a pi
        if th > np.pi:
            th = th - 2*np.pi
        if th < -np.pi:
            th = th + 2*np.pi
        self.x = x
        self.y = y
        self.th = th
        #Obtenemos el giro deseado para alinearnos
        thi = np.arctan2(self.yt-yin,self.xt-xin)
        if thi > np.pi:
            thi = th - 2*np.pi
        if thi < -np.pi:
            thi = thi + 2*np.pi
        #Obtenemos el error de giro
        ther = thi - self.th
        self.eth = ther
        #Obtenemos la distancia euclidiana al punto final
        edis = np.sqrt((self.xt - self.x)**2 + (self.yt - self.y)**2)
        #Constantes de control para el giro y avanzar hacia el punto
        kd,kth = 1,1
        if ther > np.pi:
            ther = th - 2*np.pi
        if ther < -np.pi:
            ther = thi + 2*np.pi
        w = (ther*kth)
        v = kd * edis
        #Filtro de saturacion
        if v > self.v_max:
                v = self.v_max
        elif v < -1*self.v_max:
            v = -1*self.v_max
        #Si estamos cerca del punto paramos
        if edis < 0.4:
            v,w = 0,0
        return (v,w)

    def cal_linea(self,x0,y0,xf,yf):
        #Funcion que obtiene las constantes a,b,c para oobtener la desitancia con la linea original
        x0_inv = -xf
        y0_inv = -yf
        a,b,c = 0,0,0
        a = -(yf - y0)
        b = (xf -x0)
        c = 0
        return (a,b,c)


        
    def main(self):
        #Obtenemos el tiempo inicial
        t0 = rospy.get_rostime().to_sec()
        #Estados que controlan si seguimos pared o seguimos odometria
        estado_odom = True
        estado_esquina = False
        #Obtenemos las constantes del la linea recta deseada
        a,b,c = self.cal_linea(self.x,self.y,self.xt,self.yt)
        while not rospy.is_shutdown():
            #Si el scan no esta vacio
            if self.scan is None:
                continue
            """Obtenemos la distancia hacia enfrente a la izquierda, derecha y el rayo de 45 grados de la izquierda y derecha"""
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
            #Obtenemos la diferencial de tiempos
            dt = tf- t0
            if dt > 0.1:
                dt = 0.0
            #Hacmos la odometria todo el tiempo para saber donde estamos
            v,w = self.odometry(dt,self.xin,self.yin)
            #Obtenemos la distancia con la liea original
            dist = (np.abs(a*self.x + b*self.y + c))/(np.sqrt(a**2 + b**2))
            #Si detecatmos alfrente o a los damos mas cerca que un umbral pasamos a modo wall follower
            if (self.distance_ahead < 0.5) or (self.rayo_izq < 0.2) or (self.rayo_der< 0.05) :
                self.thin = self.th
                estado_odom = False
            #Si estamos en modo wall follower y estamos cerca de la linea original y pero hay diferencia entre tu angulo inciail y final retomamos la linea original
            if (estado_odom == False) and (dist < 0.05) and (np.abs(self.thin - self.th) > 0.1):
                estado_odom = True
                #Obtenemos los nuevos puntos iniciales para alinearnos
                self.xin = self.x
                self.yin = self.y
            #Controlamos si seguimos la pared o no
            if estado_odom == False:
                print("Wall follow")
                v,w = self.follow_left_hand_wall()
            else:
                print("Odom follow")
            #Publicamos la velocidad
            msg = Twist()
            msg.angular.z = w
            msg.linear.x = v
            self.vel_pub.publish(msg)
            t0 = tf
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
    #Mapeamos del menor al mayor partiendo el abanico 720 paor (self.rayo_izq < 0.25)or (self.rayo_der < 0.25)rtes
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
    #Obtenemos el angulo de inclinacion del robot conforme al muro
    alpha = np.arctan2((-a*np.cos(np.radians(theta))-b),(-a*np.sin(np.radians(theta))))
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
    return np.mean(scan.ranges[end_index:start_index])





if __name__ == '__main__':

    rospy.init_node('Follow_right_hand_wall')
    rhw = RightHandRuleController(xt = 7, yt = 3)
    rhw.main()
    

