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


command = Int16MultiArray()
commands = SLEEPING_POS # SKIP
commands = np.concatenate([commands, STANDING_POS], 0) # SKIP

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

def eulerCb(msg):
  global euler_msg
  euler_msg = msg

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

if __name__=="__main__":
  rospy.init_node('rl_control', anonymous=True)
  rospy.Subscriber("sensor/euler", Vector3, eulerCb)
  rospy.Subscriber("sensor/imu", Imu, imuCb)
  pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)
  rate = rospy.Rate(10)
  walk_model_path=(os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/models/walk.onnx")
  stand_model_path=(os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/models/stand.onnx")
  numRotationHis = 4
  numActionHis = 4
  rotation_history = np.zeros([numRotationHis, 4])
  rotation_history[:,  3] = 1.0
  action_history = np.ones([numActionHis, 6])
  
  device='cuda'
  start_rotation = to_torch([[0, 0, 0, 1.]], device=device)
  inv_start_rot = quat_conjugate(start_rotation)

  state = 'stood'
  
  while not rospy.is_shutdown():
    # up_proj
    rotation = np.array([[-imu_msg.orientation.x, -imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]])
    torso_rotation = to_torch(rotation, device=device)
    basis_vec1 = to_torch([[0, 0, 1.]], device=device)
    up_proj = compute_up_proj(torso_rotation, inv_start_rot, basis_vec1, 2).cpu().item()

    # action
    walk_action = run_onnx_model(walk_model_path, np.concatenate([rotation_history.flatten(), action_history.flatten()], 0))
    stand_action = run_onnx_model(stand_model_path, np.concatenate([rotation_history.flatten(), action_history.flatten()], 0))

    # obs
    rotation = np.array([[-imu_msg.orientation.x, -imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]])

    if state == 'down':
      print('down -- stand_action')
      commands = np.concatenate([commands, simRad2realDeg(stand_action)], 0)
      action_history = np.concatenate([stand_action, action_history], 0)[:-1, :]
      rotation_history = np.concatenate([rotation, rotation_history], 0)[:-1, :]
      if up_proj > 0.95:
          state = 'up'
    elif state == 'up':
      print('up')
      commands = np.concatenate([commands, linspace(np.array([commands[0, :].tolist()]), SLEEPING_POS, 10)], 0)
      commands = np.concatenate([commands, linspace(SLEEPING_POS, STANDING_POS, 7)], 0)
      state = 'standing'
    elif state == 'standing':
      print('standing')
      if commands.shape[0] == 1:
          state = 'stood'
    elif state == 'stood':
      print('stood -- walk_action')
      commands = np.concatenate([commands, simRad2realDeg(walk_action)], 0)
      action_history = np.concatenate([walk_action, action_history], 0)[:-1, :]
      rotation_history = np.concatenate([rotation, rotation_history], 0)[:-1, :]
      if up_proj < 0.7:
        state = 'down'
        

    command.data = commands[0, :].tolist()
    if (commands.shape[0] - 1):
      commands = np.delete(commands, 0, 0)
    print(command.data)
    pub.publish(safeClip(command))


    rate.sleep()
