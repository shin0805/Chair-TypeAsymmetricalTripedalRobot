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
  commands = np.concatenate([commands, linspace(ROLLED_POS, SLEEPING_POS, 40)], 0) 
  commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 30)], 0)

def addTurn(is_left):
  global commands
  if is_left:
    commands = np.concatenate([commands, linspace(STANDING_POS, FORLEFT_POS1, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS1, FORLEFT_POS2, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS2, FORLEFT_POS3, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORLEFT_POS3, STANDING_POS, 15)], 0) 
  else:
    commands = np.concatenate([commands, linspace(STANDING_POS, FORRIGHT_POS1, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS1, FORRIGHT_POS2, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS2, FORRIGHT_POS3, 3)], 0) 
    commands = np.concatenate([commands, linspace(FORRIGHT_POS3, STANDING_POS, 15)], 0) 

def addStep():
  global commands
  commands = np.concatenate([commands, linspace(STANDING_POS, WALKING_POS1, 2)], 0) 
  commands = np.concatenate([commands, linspace(WALKING_POS1, WALKING_POS2, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS2, WALKING_POS3, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS3, WALKING_POS4, 2)], 0)
  commands = np.concatenate([commands, linspace(WALKING_POS4, STANDING_POS, 8)], 0)

def eulerCb(msg):
  global euler_msg
  euler_msg = msg

rospy.Subscriber("sensor/euler", Vector3, eulerCb)


i = 0
flag = 0
plan = 1
sensor = 0
rate = rospy.Rate(20)
start_time = time.time()
while not rospy.is_shutdown():
  if plan:
    if flag == 0:
      print("===== flag 0 =====")
      addStep()
      addStep()
      addStep()
    elif flag == 1:
      print("===== flag 1 =====")
      if sensor == 0:
        init_euler = euler_msg.x
      # if (abs(euler_msg.x - init_euler) < 55 or abs(euler_msg.x - init_euler) > 300) and i < 10:
      if (abs(euler_msg.x - init_euler) < 55 or abs(euler_msg.x - init_euler) > 300) and i < 3:
        print(abs(euler_msg.x - init_euler))
        sensor = 1
        i += 1
        addTurn(True)
      else:
        addStep()
        sensor = 0
        i = 0
    elif flag == 2:
      print("===== flag 2 =====")
      for i in range(5):
        addStep()
    elif flag == 3:
      print("===== flag 3 =====")
      if sensor == 0:
        init_euler = euler_msg.x
      if (abs(euler_msg.x - init_euler) < 30 or abs(euler_msg.x - init_euler) > 330) and i < 3:
        print(abs(euler_msg.x - init_euler))
        sensor = 1
        i += 0
        addTurn(False)
      else:
        commands = np.concatenate([commands, linspace(STANDING_POS, ROLLED_POS, 10)], 0)
        sensor = 0
        i = 0
    elif flag == 4:
      print("===== flag 4 =====")
      commands = np.concatenate([commands, linspace(ROLLED_POS, STANDING_POS, 2)], 0)
      commands = np.concatenate([commands, linspace(STANDING_POS, STANDING_POS, 50)], 0)
      addRise()
    elif flag == 5:
      print("===== flag 5 =====")
      for i in range(5):
        addStep()
    plan = 0

  command.data = commands[0, :].tolist()
  if (commands.shape[0] - 1):
    commands = np.delete(commands, 0, 0)
    if ((commands.shape[0] - 1) == 1):
      plan = 1
      flag += (1 - sensor)

  print(command.data)
  pub.publish(safeClip(command))
  rate.sleep()
