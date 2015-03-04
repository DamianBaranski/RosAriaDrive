#!/usr/bin/env python  
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

P=0.5; #Gain
x=0
y=0
z=0
w=0
class  RosAriaDriver():

  #Subscribe rosaria pose topic and return position and rotate
  def callback(self, data):
      global x;
      global y;
      global z;
  
      x=data.pose.pose.position.x;
      y=data.pose.pose.position.y;
      quaternion = (
          data.pose.pose.orientation.x,
          data.pose.pose.orientation.y,
          data.pose.pose.orientation.z,
          data.pose.pose.orientation.w)
      euler = tf.transformations.euler_from_quaternion(quaternion)
      z = euler[2]/3.14*180;

  #Constructor 
  def __init__(self, robot):
    self.ROBOT=robot;
    rospy.init_node('drive')
    rospy.Subscriber(self.ROBOT+"/RosAria/pose", nav_msgs.msg.Odometry, self.callback)

    self._GripperOpen  = rospy.ServiceProxy(self.ROBOT+'/RosAria/gripper_open',std_srvs.srv.Empty)
    self._GripperClose = rospy.ServiceProxy(self.ROBOT+'/RosAria/gripper_close',std_srvs.srv.Empty)
    self._GripperUp    = rospy.ServiceProxy(self.ROBOT+'/RosAria/gripper_up',std_srvs.srv.Empty)
    self._GripperDown  = rospy.ServiceProxy(self.ROBOT+'/RosAria/gripper_down',std_srvs.srv.Empty)
    self.pub = rospy.Publisher(self.ROBOT+'/RosAria/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

  #dont ready
  def calk_angle(X,Y):
    if X==0:
      if Y>0:
          kat=90;
      else:
          kat=-90;
    else:
      kat=math.atan2(abs(Y),abs(X))/3.14*180;   
      rospy.loginfo ("x=%s y=%s kat: %s",X,Y,kat)
      if X<0:
          kat=180-kat;
      if Y<0:
          kat=-kat;
    rospy.loginfo ("x=%s y=%s kat: %s",X,Y,kat)
    return kat
  pass

  #rotate robot, in global angle
  def rotate(self,angle):

      rate = rospy.Rate(10.0)
      while not rospy.is_shutdown() and abs(angle-z)>0.5:
          if angle<0 : angle2=angle+360;
          else: angle2=angle
          if z<0 : z2=z+360;
          else: z2=z
  
          self.pub.publish(geometry_msgs.msg.Twist(Vector3(0,0,0),Vector3(0,0,(angle2-z2)/50)))
          rospy.loginfo ("kat: %0.1f",z)
          rate.sleep()

  #drive robot X meter
  def goto(self,X):
      global x0;
      global y0;
      x0=x;
      y0=y;
      rospy.Subscriber(self.ROBOT+"/RosAria/pose", nav_msgs.msg.Odometry, self.callback)
      rate = rospy.Rate(10.0)
      l=math.sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y));
      while not rospy.is_shutdown() and abs(abs(X)-l)>0.005:
          l=math.sqrt((x0-x)*(x0-x)+(y0-y)*(y0-y));
          if (X>0):
             self.pub.publish(geometry_msgs.msg.Twist(Vector3((X-l)*P,0,0),Vector3(0,0,0)))
          else:
             self.pub.publish(geometry_msgs.msg.Twist(Vector3((X+l)*P,0,0),Vector3(0,0,0)))           
          rospy.loginfo ("Odleglosc: %0.2f",abs(abs(X)-l))
          rate.sleep()

  #set speed X in m/s Z in rad/s
  def setSpeed(self, X, Z):
    self.pub.publish(geometry_msgs.msg.Twist(Vector3(X,0,0),Vector3(0,0,Z)))

  #dont ready jet
  def setSpeedLR(self, L, R):
    self.pub.publish(geometry_msgs.msg.Twist(Vector3((L+R)/2,0,0),Vector3(0,0,0)))

  def GripperOpen(self):
    rospy.loginfo ("Gripper opening")
    try:
      self._GripperOpen()
    except:
       rospy.logerr("Oops!  Error opening gripper!!!")

  def GripperClose(self):
    rospy.loginfo ("Gripper closing")
    try:
      self._GripperClose()
    except:
       rospy.logerr("Oops!  Error closing gripper!!!")

  def GripperUp(self):
    rospy.loginfo ("Gripper moving up")
    try:
      self._GripperUp()
    except:
       rospy.logerr("Oops!  Error moving up gripper!!!")

  def GripperDown(self):
    rospy.loginfo ("Gripper moving down")
    try:
      self._GripperDown()
    except:
       rospy.logerr("Oops!  Error moving down gripper!!!")

