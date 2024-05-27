#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
import sys

import onnxruntime
import numpy as np

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from utils import *

from config import *

rospy.init_node('control', anonymous=True)

command = Int16MultiArray()

commands = SLEEPING_POS # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)
euler_msg = Vector3()
imu_msg = Imu()

def safeClip(command):
  tmp = np.clip(np.array(command.data), ANGLE_MIN, ANGLE_MAX).tolist()
  command.data = list(map(int, tmp))
  return command

def linspace(start, end, step):
  L = np.concatenate([start.T, end.T], 1)
  coef = np.linspace(1, 0, step).reshape(1, step)
  R =  np.concatenate([coef, coef[:, ::-1]], 0)
  return np.dot(L, R).T

def imuCb(msg):
  global imu_msg
  imu_msg = msg

def run_onnx_model(model_path, input_data):
  session = onnxruntime.InferenceSession(model_path, providers=['CPUExecutionProvider'])

  input_name = session.get_inputs()[0].name
  input_shape = session.get_inputs()[0].shape
  input_data = np.array(input_data, dtype=np.float32)
  input_data = input_data.reshape(input_shape)

  outputs = session.run(["mu"], {input_name: input_data})

  return outputs[0]

def simRad2realDeg(sim_rad):
    print(sim_rad)
    deg = -np.rad2deg(sim_rad) + 90
    real_deg = np.zeros([1, 6])
    real_deg[0, 0] = deg[0, 5]
    real_deg[0, 1] = deg[0, 4]
    real_deg[0, 2] = deg[0, 3]
    real_deg[0, 3] = deg[0, 2]
    real_deg[0, 4] = deg[0, 1]
    real_deg[0, 5] = deg[0, 0]
    return real_deg

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
    elif flag == 6:
      print("fin!")
      break
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

rospy.Subscriber("sensor/imu", Imu, imuCb)
rate = rospy.Rate(10)
model_path=(os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/models/stand.onnx")
numRotationHis = 4
numActionHis = 4
rotation_history = np.zeros([numRotationHis, 4])
rotation_history[:,  3] = 1.0
action_history = np.ones([numActionHis, 6])

# device='cuda'
device='cpu'
start_rotation = to_torch([[0, 0, 0, 1.]], device=device)
inv_start_rot = quat_conjugate(start_rotation)

state = 'stood'

while not rospy.is_shutdown():
  # action
  action = run_onnx_model(model_path, np.concatenate([rotation_history.flatten(), action_history.flatten()], 0))

  # obs
  rotation = np.array([[-imu_msg.orientation.x, -imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]])
  rotation_history = np.concatenate([rotation, rotation_history], 0)[:-1, :]
  action_history = np.concatenate([action, action_history], 0)[:-1, :]

  # up_proj
  torso_rotation = to_torch(rotation, device=device)
  basis_vec1 = to_torch([[0, 0, 1.]], device=device)
  up_proj = compute_up_proj(torso_rotation, inv_start_rot, basis_vec1, 2).cpu().item()

  if state == 'down':
    print('down (up_proj = {up_proj})')
    commands = np.concatenate([commands, linspace(STANDING_POS, EXTENTION_POS, 10)], 0) 
    commands = np.concatenate([commands, linspace(EXTENTION_POS, ROLLED_POS, 2)], 0) 
    commands = np.concatenate([commands, linspace(ROLLED_POS, SLEEPING_POS, 25)], 0) 
    commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 8)], 0)
    state = 'standing'
  elif state == 'standing':
    print('standing')
    if commands.shape[0] == 1:
        state = 'stood'
  elif state == 'stood':
    if -0.1 <= up_proj < 0.7:
      state = 'down'
    print(f'stood (up_proj = {up_proj})')
      

  command.data = commands[0, :].tolist()
  if (commands.shape[0] - 1):
    commands = np.delete(commands, 0, 0)
  print(command.data)
  pub.publish(safeClip(command))


  rate.sleep()
