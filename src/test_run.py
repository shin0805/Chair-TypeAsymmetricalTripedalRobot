#!/usr/bin/env python
import os
import subprocess
import time

connect_walk = os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/src/connect_walk.py"
connect_stand = os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/src/connect_stand.py"
rl_walk = os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/src/rl_walk.py"
rl_stand = os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/src/rl_stand.py"
init = os.environ['HOME'] + "/Chair-TypeAsymmetricalTripedalRobot/src/init.py"
paths = [connect_walk, connect_stand, rl_walk, rl_stand, init]

flag = True
while flag:
  try:
    command = int(input("1:connect_walk, 2:conncet_stand, 3:rl_walk, 4: rl_stand, 5: init, 0: exit\n"))
    flag = command
  except:
    flag = False

  if flag:
    process = subprocess.Popen(paths[command-1])
    try:
      if command != 5: 
        input("Enter to stop")
      else:
        time.sleep(1)
    except KeyboardInterrupt:
      pass
    process.terminate()
  else: 
    print("exit")

