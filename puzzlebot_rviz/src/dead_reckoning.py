#!/usr/bin/env python

#Santiago Ortiz Suzarte 		   A01750402
#Daniel Fuentes Castro			A01750425
#Leonardo Gracida Munoz 		A01379812
#Importamos las librerias necesarias
import tf, rospy
import numpy as np
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, Point, Quaternion,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import numpy as np
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
x, y, q = 0, 0, -1.57 #Declaramos las posiciones iniciales del localizador con la mejor estimacion
"""Las declaramos en esos valores, ya que es la posicion en la que inicia la posicion en el espacio en la que inicia el robot en gazebo"""
x_gaze,y_gaze,q_gaze = 0.0,0.0,[0,0,0,0] #Creamos las variables de localizacion del robot en gazebo
r, l = 0.05, 0.188 #Radio y  y largo del robot
v, w = 0.0, 0.0

#Callback que extrae la informacion de velocidad del topioc /cmd_vel
def callback_top(msg):
    global v, w
    v=msg.linear.x
    w=msg.angular.z

#Callback para llamar el servicio de reset el inciar este codigo
def callback_ser(req):
    global x, y, q
    x, y, q = 0.0, 0.0, 0.0
    return EmptyResponse()

#Callback que descompone y extrae informacion de posicion y angulo del robot en gazebo
def callback_gazebo_pose(data):
	global x_gaze,y_gaze,q_gaze
	x_gaze = data.pose[-1].position.x
	y_gaze = data.pose[-1].position.y
	q_gaze = [data.pose[-1].orientation.x,data.pose[-1].orientation.y,data.pose[-1].orientation.z,data.pose[-1].orientation.w]


#Funcion que corre tanto la mejor posicion como publica la poisicion del robot de gazebo en RVIZ
def node():
	t0 = rospy.get_rostime().to_sec()
	global x, y,q
	#Declaramos el tamano de la diferencial de tiempo
	T = 1.0/fs
	t0 = rospy.Time.now()
	#Extraemos los angulos de inclinacion iniciales o anteriores para el calculo de la covarianza
	th_k_1 = q
	s_th_k_1 = euler_from_quaternion(q_gaze)[2]
	#Creamos la matriz de zeros de 3 y 6 dimensioness
	cov_1 = np.zeros(3)
	cov_com = np.zeros((6,6))
	while not rospy.is_shutdown():
		#Hacemos el calculo de la mejor aproximacion
		x += (T)*(v)*np.cos(q)
		y += (T)*(v)*np.sin(q)
		q += (T)*(w)
		#Transformamos el angulo de giro de euler a quaternion
		qRota = tf.transformations.quaternion_from_euler(0,0,q)
		cTime = rospy.Time.now()
		#Publicamos la velocidad de las llantas basandonos en la velocidad angular y lineal actual
		pWl.publish((v - 0.5*l*w)/r)
		pWr.publish((v + 0.5*l*w)/r)
		#Publicamosl posicion del robot de RVIZ conforme a la de gazebo
		tb.sendTransform([x_gaze,y_gaze,0], q_gaze, cTime, "base_link", "map")
		#Creamos el objeto para publicar estado de los joints de las llantas
		js = JointState()
		js.name = ["right_wheel_joint","left_wheel_joint"]
		t = cTime-t0
		js.position = [((v + 0.5*l*w)/r)*t.to_sec(),((v - 0.5*l*w)/r)*t.to_sec()]
		js.header.stamp = cTime
		#Publicamos el giro de las llantas
		pJS.publish(js)
		#Armamos le jacobiano con el angulo anterior de inclinacion de la mejor aproximacion
		H_k = np.array([[1, 0, -T*v*np.sin(th_k_1)],
						[0, 1, T*v*np.cos(th_k_1)],
						[0, 0, 1]])
		#Dalcarmos el valor de la variable kr y kl
		kr = 95
		kl = 95
		#Declaramos La estructura de las dos matrices que conforma la matriz de error no deterministica
		trian_k = np.array([[kr*np.abs((v + 0.5*l*w)/r), 0],
							[0, kl*np.abs((v - 0.5*l*w)/r)]])
		inv_trian_w_k = 0.5*r*T*np.array([[np.cos(s_th_k_1),np.cos(s_th_k_1)],
										[np.sin(s_th_k_1),np.sin(s_th_k_1)],
										[(2/l),(-2/l)]])
		#Obtenemos la matriz de error deterministica
		Q_k = np.dot(inv_trian_w_k,np.dot(trian_k,inv_trian_w_k.T))
		#Obtenemos la matriz de covarianza
		cov = np.dot(H_k,np.dot(cov_1,H_k.T))+Q_k
		#Cambiamos los valores de la matriz de covarianza de tres dimensiones
		cov_com[0,0] = cov[0,0]
		cov_com[1,0] = cov[1,0]
		cov_com[5,0] = cov[2,0]
		cov_com[0,1] = cov[0,1]
		cov_com[1,1] = cov[1,1]
		cov_com[5,1] = cov[2,1]
		cov_com[0,5] = cov[0,2]
		cov_com[1,5] = cov[1,2]
		cov_com[5,5] = cov[2,2]
		cov_com[2,2] = 0.0001
		#Publicamos el mensaje de odometria
		locationCovariance = Odometry()
		locationCovariance.header.frame_id = "map"
		locationCovariance.header.stamp = cTime
		locationCovariance.child_frame_id = "base_link"
		locationCovariance.pose.pose.position = Point(x,y,r)
		locationCovariance.pose.pose.orientation = Quaternion(qRota[0],qRota[1],qRota[2],qRota[3])
		locationCovariance.pose.covariance = cov_com.reshape(1,36)[0]
		locationCovariance.twist.twist.linear.x = v
		locationCovariance.twist.twist.linear.y = 0
		locationCovariance.twist.twist.linear.z = 0
		locationCovariance.twist.twist.angular.x = 0
		locationCovariance.twist.twist.angular.y = 0
		locationCovariance.twist.twist.angular.z = w
		locationCovariance.twist.covariance = np.zeros((6,6)).reshape(1,36)[0]
		pPoseCov.publish(locationCovariance)
		#Hacemos que los valores actuales se vuelvan los anteriores para la siguiente interacion
		cov_1 = cov
		th_k_1 = q
		s_th_k_1 = euler_from_quaternion(q_gaze)[2]
		rate.sleep()

if __name__ == '__main__':
    try:
		#iniciamos el bodo
		rospy.init_node('puzzlebot_kinematic_model')
		#Creamos el objeto para llamar el servicio de EMpty
		ser = rospy.Service("/reset", Empty, callback_ser)
		#Declaramos el objeto del broadcaster
		tb = tf.TransformBroadcaster()
		#Declaramos los publicadores necesarios
		pPoseCov = rospy.Publisher('/odom', Odometry, queue_size=10)
		pWl   = rospy.Publisher('/rviz/wl', Float32, queue_size=10)
		pWr   = rospy.Publisher('/rviz/wr', Float32, queue_size=10)
		pJS = rospy.Publisher('/joint_states', JointState,queue_size=10)
		#Declaramos los suscribers necesarios
		rospy.Subscriber("/cmd_vel", Twist, callback_top)
		rospy.Subscriber("/gazebo/model_states", ModelStates, callback_gazebo_pose)
		#Numero de mensajes por minuto
		fs = 50
		rate = rospy.Rate(fs)
		node()
    except rospy.ROSInterruptException:
    	pass

