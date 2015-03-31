#!/usr/bin/env python 
from time import sleep 
from drive import RosAriaDriver 

robot=RosAriaDriver('/PIONIER1')

print(robot.ReadLaser()); 

robot.SetSpeed(0.5,0,3);
robot.StopRobot();
