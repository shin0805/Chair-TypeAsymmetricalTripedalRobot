#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time

import rospy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Vector3
import numpy as np

from config import *

rospy.init_node('control', anonymous=True)

command = Int16MultiArray()

commands = SLEEPING_POS # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)
euler_msg = Vector3()

def safeClip(command):
  tmp = np.clip(np.array(command.data), ANGLE_MIN, ANGLE_MAX).tolist()
  command.data = list(map(int, tmp))
  return command

def linspace(start, end, step):
  L = np.concatenate([start.T, end.T], 1)
  coef = np.linspace(1, 0, step).reshape(1, step)
  R =  np.concatenate([coef, coef[:, ::-1]], 0)
  return np.dot(L, R).T

def addRise():
  global commands
  commands = np.concatenate([commands, linspace(STANDING_POS, EXTENTION_POS, 10)], 0) 
  commands = np.concatenate([commands, linspace(EXTENTION_POS, ROLLED_POS, 2)], 0) 
  commands = np.concatenate([commands, linspace(ROLLED_POS, SLEEPING_POS, 10)], 0) 
  commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 7)], 0)

def addStep():
  global commands
  commands = np.concatenate([commands, linspace(STANDING_POS, WALKING_POS1, 2)], 0) 
  commands = np.concatenate([commands, linspace(WALKING_POS1, WALKING_POS2, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS3, WALKING_POS4, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS4, STANDING_POS, 8)], 0)

def eulerCb(msg):
  global euler_msg
  euler_msg = msg

rospy.Subscriber("sensor/euler", Vector3, eulerCb)

addRise()

rate = rospy.Rate(20)
start_time = time.time()
while not rospy.is_shutdown():
  command.data = commands[0, :].tolist()
  if (commands.shape[0] - 1):
    commands = np.delete(commands, 0, 0)

  print(command.data)
  pub.publish(safeClip(command))
  rate.sleep()
