#!/usr/bin/env python
import rospy
import numpy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class Tacka():
    def __init__(self,x,y):
        self.x = x
        self.y = y


cilj = Tacka(5,5)
pozicija = Tacka(0,0)

ranges=[0] * 360
orijentacija = 0
prepreka = False
w_y = 0
v_x = 0



def odrediPoziciju(data):
    global orijentacija
    global pozicija
    pozicija.x = data.pose.pose.position.x
    pozicija.y = data.pose.pose.position.y
    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    orijentacija=euler[2]	
    
# Funkcija za uzimanje podataka sa senzora         
def podaciSaSenzora(data):
    global ranges    
    ranges=data.ranges

def odrediBrzine(pozicija_robota, pozicija_cilja):
    global prepreka
    global w_z
    global v_x
   
    matrica_rotacije = numpy.matrix([[numpy.cos(orijentacija), -numpy.sin(orijentacija)], [numpy.sin(orijentacija), numpy.cos(orijentacija)]])
    priv_matr = numpy.matrix([[cilj.x - pozicija.x, cilj.y - pozicija.y]])
    A = priv_matr * matrica_rotacije
    ugao_cilja2 = numpy.arctan2(A[0,1], A[0,0]) #u koordinatnom sistemu robota
    udaljenost = math.hypot(cilj.x-pozicija.x,cilj.y-pozicija.y)

    if udaljenost < 0.1:
        v_x = 0
        w_z = 0
        return v_x, w_z
    if (udaljenost>1.5 and (ranges[119]<1.5 or ranges[180]<1.5 or ranges[239]<1.5 or ranges[0]<0.8 or ranges[359]<0.8)):
	    prepreka = True
    else:
	    prepreka = False

    if prepreka == True:
        if (ranges[239]<ranges[180] and ranges[239]<ranges[119]):
            if ranges[239]>0.8:
                w_z=-0.1
                v_x=0.3
            else:
                w_z=-0.5
                v_x=0.2	    	
        if (ranges[180]<ranges[239] and ranges[180]<ranges[119]):
            if ranges[180]>0.8:
                w_z=0.1
                v_x=0.3
            else:
                w_z=0.5
                v_x=0.2
        if (ranges[119]<ranges[180] and ranges[119]<ranges[239]):
            if ranges[119]>0.8:
                w_z=0.1
                v_x=0.3
            else:
                w_z=0.5
                v_x=0.2
    else:
        if (ugao_cilja2>0.05 and ugao_cilja2<numpy.pi and udaljenost>1.5):
	        w_z=0.5
	        v_x=0.3
        elif (ugao_cilja2<-0.05 and ugao_cilja2>-numpy.pi and udaljenost>1.5):
	        w_z=-0.5
	        v_x=0.3
        elif (udaljenost>1.5):
	        w_z=0
	        v_x=0.3
	    
    return v_x, w_z

def postavljanje_brzine():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, odrediPoziciju)
    rospy.Subscriber("base_scan", LaserScan, podaciSaSenzora)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        brzina = Twist()
        v_x, w_z = odrediBrzine(pozicija, cilj)
        brzina.linear.x = v_x
        brzina.angular.z = w_z
        pub.publish(brzina)
        rate.sleep()

if __name__ == '__main__':
    try:
        postavljanje_brzine()
    except rospy.ROSInterruptException:
        pass
