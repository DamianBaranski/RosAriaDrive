#!/usr/bin/env python 
# -*- coding: utf-8 -*-

## @class RosAriaDriver
#  
#  Klasa ułatwiająca połączenie z robotem Pioneer
#
#  @author Damian Barański \n
#  Data : 25/02/2015 \n
#  Oprogramowanie na licencji GNU GPL

import roslib
import rospy
import tf
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

P=0.5; #Gain

class  RosAriaDriver():


  #Subscribe rosaria pose topic and return position and rotate
  def _callback_pose(self, data):
      self._x=data.pose.pose.position.x;
      self._y=data.pose.pose.position.y;
      quaternion = (
          data.pose.pose.orientation.x,
          data.pose.pose.orientation.y,
          data.pose.pose.orientation.z,
          data.pose.pose.orientation.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      self._z = euler[2]/3.14*180;

  ## Konstruktor.
  #  @param self Wskaźnik na obiekt.
  #  @param robot Nazwa robota.
  def __init__(self, robot):
    self._ROBOT=robot;
    self._x=0;
    self._y=0;
    self._z=0;
    rospy.init_node('drive')
    rospy.Subscriber(self._ROBOT+"/RosAria/pose", nav_msgs.msg.Odometry, self._callback_pose)

    self._GripperOpen  = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_open',std_srvs.srv.Empty)
    self._GripperClose = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_close',std_srvs.srv.Empty)
    self._GripperUp    = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_up',std_srvs.srv.Empty)
    self._GripperDown  = rospy.ServiceProxy(self._ROBOT+'/RosAria/gripper_down',std_srvs.srv.Empty)
    self._pub = rospy.Publisher(self._ROBOT+'/RosAria/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

  #dont ready
#  def calk_angle(X,Y):
#    if X==0:
#      if Y>0:
#          kat=90;
#      else:
#          kat=-90;
#    else:
#      kat=math.atan2(abs(Y),abs(X))/3.14*180;   
#      rospy.loginfo ("x=%s y=%s angle: %s",X,Y,kat)
#      if X<0:
#          kat=180-kat;
#      if Y<0:
#          kat=-kat;
#    rospy.loginfo ("x=%s y=%s angle: %s",X,Y,kat)
#    return kat
#  pass

  #rotate robot, in global angle
  ## Powoduje obrót robota.
  #  @param self Wskaźnik na obiekt.
  #  @param angle Kąt w którym ma się znaleźć robot.
  def Rotate(self,angle):

      rate = rospy.Rate(10.0)
      while not rospy.is_shutdown() and abs(angle-self._z)>0.5:

        #  if(self.pub.get_num_connections()==0):
        #      rospy.logerr("Oops!  Error connect with robot")
        #      return 

          if angle<0 : angle2=angle+360;
          else: angle2=angle
          if self._z<0 : z2=self._z+360;
          else: z2=self._z
  
          self.pub.publish(geometry_msgs.msg.Twist(Vector3(0,0,0),Vector3(0,0,(angle2-z2)/50)))
          rospy.loginfo ("Angle: %0.1f",z)
          rate.sleep()

  #drive robot X meter
  ## Powoduje jazdę robota.
  #  @param self Wskaźnik na obiekt.
  #  @param X Ilość metrów do przejechania.

  def GoTo(self,X):
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
    while T>0:
      self._pub.publish(geometry_msgs.msg.Twist(Vector3(X,0,0),Vector3(0,0,Z)))
      sleep(0.1);
      T=T-0.1

  ## Zadanie prędkości lewego i prawego koła .
  #  @param self Wskaźnik na obiekt.
  #  @param L Prędkość lewego koła w [m/s].
  #  @param R Prędkość prawego koła w[m/s].
  #  @param T Czas trwania w [s].

  def SetSpeedLR(self, L, R, T):
    SpeedX = (L+R)/2.0
    SpeedZ = (R-L)/0.185 #polowa roztawu osi

    while T>0:
      self._pub.publish(geometry_msgs.msg.Twist(Vector3(SpeedX,0,0),Vector3(0,0,SpeedZ)))
      sleep(0.1);
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

