#!/usr/bin/env python 
# -*- coding: utf-8 -*-

## @class RosAriaDriver
#  
#  Klasa ułatwiająca połączenie z robotem Pioneer
#
#  @author   Damian Barański \n
#  @date     25/02/2015 \n
#  @license  Projekt rozwijany na licencji GNU GPL

import roslib
import rospy
import tf
import struct
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
import nav_msgs.msg
from nav_msgs.msg import Odometry
import std_srvs.srv
from std_srvs.srv import Empty
import math
from time import sleep

P=1.5; #Gain
MaxAngleSpeed=0.8;

class  RosAriaDriver():
    #Subscribe rosaria pose topic and return position and rotate
  def _callback_pose(self, data):
    self._x=data.pose.pose.position.x-self._x0;
    self._y=data.pose.pose.position.y-self._y0;

    quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    self._z = divmod(euler[2]/3.14*180-self._z0+180,360)[1]-180;
    self._ready=1;


  def _callback_sonar(self, data):
    #byte to float32
    i=0;
    self._sonar=[];
    for i in range(0, data.width*12, 12):
      x=struct.unpack('f', '%c%c%c%c' % (data.data[0+i], data.data[1+i], data.data[2+i], data.data[3+i] ))[0];
      y=struct.unpack('f', '%c%c%c%c' % (data.data[4+i], data.data[5+i], data.data[6+i], data.data[7+i] ))[0];
      self._sonar.append([x, y])

    self._SonarReady=1;


  ## Konstruktor.
  #  @param self Wskaźnik na obiekt.
  #  @param robot Nazwa robota.
  def __init__(self, robot):
    self._ROBOT=robot;
    self._x=0;
    self._y=0;
    self._z=0;
    self._x0=0;
    self._y0=0;
    self._z0=0;
    self._ready=0;

    self._sonar = [];
    self._SonarReady=0;
   
    rospy.init_node('drive')
    rospy.Subscriber(self._ROBOT+"/RosAria/pose", nav_msgs.msg.Odometry, self._callback_pose)
    rospy.Subscriber(self._ROBOT+"/RosAria/sonar_pointcloud2",  sensor_msgs.msg.PointCloud2, self._callback_sonar)

    self._GripperOpen  = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_open',std_srvs.srv.Empty)
    self._GripperClose = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_close',std_srvs.srv.Empty)
    self._GripperUp    = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_up',std_srvs.srv.Empty)
    self._GripperDown  = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_down',std_srvs.srv.Empty)
    self._pub = rospy.Publisher(self._ROBOT+'/RosAria/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

  #rotate robot, in global angle
  ## Powoduje obrót robota.
  #  @param self Wskaźnik na obiekt.
  #  @param angle Kąt w którym ma się znaleźć robot.
  def Rotate(self,angle):
      #wait for data from robot
      while self._ready==0:
        pass

      z2=divmod(self._z,360)[1];        
      angle2=divmod(angle,360)[1];


      rate = rospy.Rate(10.0)
      while not rospy.is_shutdown() and abs(angle2-z2)>1.0:

        #  if(self.pub.get_num_connections()==0):
        #      rospy.logerr("Oops!  Error connect with robot")
        #      return 

          z2=divmod(self._z,360)[1];        

          if (angle2-z2)<0 :
            Dir=-1;
          else:
            Dir=1;
      
          if abs(angle2-z2)>180:
            Dir=-Dir
            
          self._pub.publish(geometry_msgs.msg.Twist(Vector3(0,0,0),Vector3(0,0,Dir*min(abs(angle2-z2)/50,MaxAngleSpeed))))
          rospy.loginfo ("Angle: %0.1f",self._z)
          rate.sleep()

  #drive robot X meter
  ## Powoduje jazdę robota.
  #  @param self Wskaźnik na obiekt.
  #  @param X Ilość metrów do przejechania.

  def GoTo(self,X):
      #wait for data from robot
      while self._ready==0:
        pass

      x0=self._x;
      y0=self._y;
      rospy.Subscriber(self._ROBOT+"/RosAria/pose", nav_msgs.msg.Odometry, self._callback_pose)
      rate = rospy.Rate(10.0)
      l=math.sqrt((x0-self._x)*(x0-self._x)+(y0-self._y)*(y0-self._y));
      while not rospy.is_shutdown() and abs(abs(X)-l)>0.005:
          #if(self.pub.get_num_connections()==0):
          #    rospy.logerr("Oops!  Error connect with robot")
          #    return 

          l=math.sqrt((x0-self._x)*(x0-self._x)+(y0-self._y)*(y0-self._y));
          if (X<0):
             self._pub.publish(geometry_msgs.msg.Twist(Vector3((X+l)*P,0,0),Vector3(0,0,0)))           
          else:
             self._pub.publish(geometry_msgs.msg.Twist(Vector3((X-l)*P,0,0),Vector3(0,0,0)))

          rospy.loginfo ("Distance: %0.2f",abs(abs(X)-l))
          rate.sleep()

  #set speed X in m/s Z in rad/s
  ## Zadanie prędkości jazdy i obrotu.
  #  @param self Wskaźnik na obiekt.
  #  @param X Prędkość postępowa w [m/s].
  #  @param Z Prędkość obrotu w [rad/s].
  #  @param T Czas trwania w [s].
  def SetSpeed(self, X, Z, T):
    self._pub.publish(geometry_msgs.msg.Twist(Vector3(X,0,0),Vector3(0,0,Z)))
    while T>0:
      sleep(0.1);
      self._pub.publish(geometry_msgs.msg.Twist(Vector3(X,0,0),Vector3(0,0,Z)))
      T=T-0.1

  ## Zadanie prędkości lewego i prawego koła .
  #  @param self Wskaźnik na obiekt.
  #  @param L Prędkość lewego koła w [m/s].
  #  @param R Prędkość prawego koła w[m/s].
  #  @param T Czas trwania w [s].

  def SetSpeedLR(self, L, R, T):
    SpeedX = (L+R)/2.0
    SpeedZ = (R-L) /0.185 #polowa roztawu osi
    self._pub.publish(geometry_msgs.msg.Twist(Vector3(SpeedX,0,0),Vector3(0,0,SpeedZ)))
    while T>0:
      sleep(0.1);
      self._pub.publish(geometry_msgs.msg.Twist(Vector3(SpeedX,0,0),Vector3(0,0,SpeedZ)))
      T=T-0.1

  ## Otwarcie chwytaka.
  #  @param self Wskaźnik na obiekt.
  def GripperOpen(self):
    rospy.loginfo ("Gripper opening")
    try:
      self._GripperOpen()
    except:
       rospy.logerr("Oops!  Error opening gripper!!!")

  ## Zamknięcie chwytaka.
  #  @param self Wskaźnik na obiekt.
  def GripperClose(self):
    rospy.loginfo ("Gripper closing")
    try:
      self._GripperClose()
    except:
       rospy.logerr("Oops!  Error closing gripper!!!")

  ## Podniesienie chwytaka.
  #  @param self Wskaźnik na obiekt.
  def GripperUp(self):
    rospy.loginfo ("Gripper moving up")
    try:
      self._GripperUp()
    except:
       rospy.logerr("Oops!  Error moving up gripper!!!")

  ## Opuszczenie chwytaka.
  #  @param self Wskaźnik na obiekt.
  def GripperDown(self):
    rospy.loginfo ("Gripper moving down")
    try:
      self._GripperDown()
    except:
       rospy.logerr("Oops!  Error moving down gripper!!!")

  ## Metoda zwracająca pozycje robota
  #  @param self Wskaźnik na obiekt.
  #  @return x[m], y[m], z[stopnie].
  
  def GetPose(self):
    while self._ready==0:
      pass
    return (self._x-self._x0, self._y-self._y0, divmod(self._z-self._z0+180,360)[1]-180)

  ## Zeruje pozycję robota
  #  @param self Wskaźnik na obiekt.
  
  def ResetPose(self):
    self._ready=0;
    while self._ready==0:
      pass
    self._x0=self._x;
    self._y0=self._y;
    self._z0=self._z;

  ## Zwraca dane z sonarów
  #  @param self Wskaźnik na obiekt.
  #  @return tabicę [[x,y],[x,y],...].

  def ReadSonar(self):
    self._SonarReady=0;
    while self._SonarReady==0:
      pass
    return self._sonar;


  ## Informacja.
  #  @param self Wskaźnik na obiekt.
  def About(self):
    print("\n\n\n")
    print("Biblioteka do obsługi robotów pioneer w python")
    print("Autor    : Damian Barański")
    print("Data     : 25-02-2015")
    print("Licencja : GNU GPL")
    print("Kontakt  : damian.baranski@pwr.wroc.pl")
    print("\n\n\n")

