#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
import time
import sys

rospy.init_node('control', anonymous=True)

data = Int16MultiArray()

pub = rospy.Publisher('servo/command', Int16MultiArray, queue_size=1)

step = 0
st = time.time()
rate = rospy.Rate(10)
check_port = int(sys.argv[1])
data.data = [90, 90, 90, 90, 90, 90]
while not rospy.is_shutdown():
  tmp = [90, 90, 90, 90, 90, 90]
  if step % 4 == 0:
    tmp[check_port] = 0
    print('0')
  elif step % 4 == 1:
    tmp[check_port] = 90
    print('90')
  elif step % 4 == 2:
    tmp[check_port] = 180
    print('180')
  elif step % 4 == 3:
    tmp[check_port] = 90
    print('90')
  data.data = tmp

  pub.publish(data)

  rate.sleep()

  if (time.time() - st) >= 1:
    step += 1
    st = time.time()
