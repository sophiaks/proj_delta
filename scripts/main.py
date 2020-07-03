#! /usr/bin/env python
# -*- coding:utf-8 -*-

# Sugerimos rodar com:
# roslaunch turtlebot3_gazebo  turtlebot3_empty_world.launch 
#
# Esta solução pode ser vista em: https://youtu.be/GKDZPcwf2WU


from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
import math
import time
from tf import transformations


x = None
y = None
acabou = False
rad_z = 0.0

contador = 0
pula = 50

def recebe_odometria(data):
    global x
    global y
    global rad_z
    global contador

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    quat = data.pose.pose.orientation
    lista = [quat.x, quat.y, quat.z, quat.w]
    angulos_rad = transformations.euler_from_quaternion(lista)
    rad_z = angulos_rad[2]
    angulos = np.degrees(angulos_rad)    

    if contador % pula == 0:
        print("Posicao (x,y)  ({:.2f} , {:.2f}) + angulo {:.2f}".format(x, y,angulos[2]))
    contador = contador + 1

v_ang = 0.1
v_lin = 0.1

def gira360(pub):
    print("Comecando a girar")
    vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,0.3))
    pub.publish(vel_rot)
    delta_t = abs(2*math.pi)/0.3
    rospy.sleep(delta_t)
    print("parou de girar")

    #   CHECAR SE TODOS OS ITENS DA LISTA FORAM ENCONTRADOS

    #   SE SIM, ACABOU == TRUE

    #   fAZER IF AQUI PRA PARAR SE ACABOU == TRUE

def go_to(x1, y1, pub):
    x0 = x # Vai ser atualizado via global e odometria em um thread paralelo
    y0 = y # global e odometria (igual ao acima)
    delta_x = x1 - x0
    delta_y = y1 - y0

    h = math.sqrt(delta_x**2 + delta_y**2) # Distancia ate o destino. Veja 
    # https://web.microsoftstream.com/video/f039d50f-3f6b-4e01-b45c-f2bffd2cbd84

    print("Goal ", x1,",",y1)

    ang_goal = math.atan2(delta_y,delta_x)  
    ang_atual = rad_z # rad_z muda automaticamente via global e odometria
    dif_ang = ang_goal - ang_atual
    delta_t = abs(dif_ang)/v_ang
    # Twist
    if dif_ang > 0.0:
        vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,v_ang))
    elif dif_ang <=0:
        vel_rot = Twist(Vector3(0,0,0), Vector3(0,0,-v_ang))    
    # publish
    pub.publish(vel_rot)
    # sleep
    rospy.sleep(delta_t)
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
    pub.publish(zero)
    rospy.sleep(0.1)
    # Translacao
    delta_t = h/v_lin
    linear = Twist(Vector3(v_lin,0,0), Vector3(0,0,0))
    pub.publish(linear)
    rospy.sleep(delta_t)
    pub.publish(zero)
    rospy.sleep(0.1)  
    x0 = x
    y0 = y
    delta_x = x1 - x0
    delta_y = y1 - y0
    h = math.sqrt(delta_x**2 + delta_y**2) # Distancia ate o destino. Veja 

        

if __name__=="__main__":

    rospy.init_node("odometry", anonymous=True)

    # Velocidades
    zero = Twist(Vector3(0,0,0), Vector3(0,0,0))
    anti = Twist(Vector3(0,0,0), Vector3(0,0,-0.3))
    hor = Twist(Vector3(0,0,0), Vector3(0,0,0.3))

    linear = Twist(Vector3(v_lin,0,0), Vector3(0,0,0))
    

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 3 )

    ref_odometria = rospy.Subscriber("/odom", Odometry, recebe_odometria)

    velocidade_saida.publish(zero)
    rospy.sleep(1.0) # contorna bugs de timing    

    while not rospy.is_shutdown():
        #SE O ROB^O ESTIVER BATENDO NAS COISAS DIMINUIR A VELOCIDADE
        rospy.sleep(0.5)    

        # qnd e 5.5 ele nao ajusta bem e bate na parede
        # com 5.4 ele ajusta o angulo pra 0.01 e melhora o resultado
        go_to(5, -5.40, velocidade_saida)
        go_to(10, -5.40, velocidade_saida)
        go_to(14, -5.50, velocidade_saida)
        #   Entra na 400 e alguma coisa
        go_to(14, -2.5, velocidade_saida)
        
        #   Gira na 400 e alguma coisa
        gira360(velocidade_saida)

        go_to(16.7, -2, velocidade_saida)
        
        gira360(velocidade_saida)

        go_to(14, -3, velocidade_saida)
        go_to(14, -5.5, velocidade_saida)  
    
        #   Vai pra metade do corredor (mais estabilidade)

        go_to(8.5, -5.5, velocidade_saida) 

        #   Vai pro ponto inicial (caminho dividido em 2 pra estabilidade)
        go_to(4, -5.50, velocidade_saida)
        go_to(0, -5.50, velocidade_saida)

        #   Entra no L3
        go_to(0, -3.50, velocidade_saida)
        go_to(3, -3.50, velocidade_saida)

        gira360(velocidade_saida)

        #   Entra na 403
        go_to(0, -3.50, velocidade_saida)
        go_to(0, 0, velocidade_saida)
        go_to(0, 2.50, velocidade_saida)

        gira360(velocidade_saida)

        #   Entra na 404
        go_to(0, 1.50, velocidade_saida)
        go_to(1, 1.50, velocidade_saida)
        go_to(1, 3.50, velocidade_saida)

        gira360(velocidade_saida)

        #   Entra no L2
        go_to(0, 0.0, velocidade_saida)
        go_to(-2.0, -1.0, velocidade_saida)
        go_to(-4.5, -1.0, velocidade_saida)
        go_to(-7.0, -1.0, velocidade_saida)
        #   Gira no meio do L2
        gira360(velocidade_saida)

        #   Sai do L2 e vai pro corredor
        go_to(-9.0, -1.0, velocidade_saida)
        go_to(-11.0, -1.0, velocidade_saida)
        go_to(-13.5, -1.0, velocidade_saida)
        go_to(-16, -1.0, velocidade_saida)
        go_to(-18, 0.0, velocidade_saida)

        #   Entra na sala de alguma coisa termica
        go_to(-18, 3.0, velocidade_saida)
        #   Gira na porta do lab
        gira360(velocidade_saida)

        #   Sai do lab chique, vai pro de materiais
        go_to(-18, 0.0, velocidade_saida)
        go_to(-18, -4.5, velocidade_saida)
        go_to(-20, -4.5, velocidade_saida)
        #   Gira na porta do lab de materiais
        gira360(velocidade_saida)

        #   Vai pro meio do corredor perto do L2
        go_to(-18.0, -4.5, velocidade_saida)
        go_to(-16.0, -4.5, velocidade_saida)
        go_to(-14.0, -4.5, velocidade_saida)
        go_to(-12.5, -4.5, velocidade_saida)
        go_to(-10.0, -4.5, velocidade_saida)
        go_to(-8.5, -4.5, velocidade_saida)
        go_to(-6.5, -4.5, velocidade_saida)
        #   Gira no meio do corredor
        gira360(velocidade_saida)

        #   Sai pela porta perto do L2
        go_to(-4.5, -5.5, velocidade_saida)
        go_to(-2.5, -5.5, velocidade_saida)
        go_to(-1.0, -5.5, velocidade_saida)
        go_to(0.5, -5.5, velocidade_saida)
        go_to(0.5, -8.5, velocidade_saida)

        #   Vai pro cantinho com as duas portas
        go_to(2.5, -8.5, velocidade_saida)
        go_to(4.5, -8.5, velocidade_saida)
        go_to(6.5, -8.5, velocidade_saida)
        go_to(8.0, -8.5, velocidade_saida)
        go_to(9.5, -8.5, velocidade_saida)
        #   Gira
        gira360(velocidade_saida)
        
        acabou = True

        while acabou == True:
            velocidade_saida.publish(zero)

        rospy.sleep(1.0)