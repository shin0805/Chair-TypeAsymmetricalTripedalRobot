import numpy as np

ANGLE_MIN = 90 - 50
ANGLE_MAX = 90 + 50

STANDING_POS = np.array([[90, 80, 90, 100, 90, 100]])
SLEEPING_POS = np.array([[130, 40, 50, 140, 90, 140]])

EXTENTION_POS = np.array([[90, 80, 90, 150, 40, 90]])
ROLLED_POS = np.array([[130, 80, 40, 90, 150, 140]])

FORRIGHT_POS1 = np.array([[90, 80, 60, 100, 90, 100]])
FORRIGHT_POS2 = np.array([[90, 80, 60, 100, 60, 100]])
FORRIGHT_POS3 = np.array([[60, 80, 60, 100, 60, 100]])

FORLEFT_POS1 = np.array([[90, 80, 90, 100, 120, 100]])
FORLEFT_POS2 = np.array([[90, 80, 90, 140, 120, 100]])
FORLEFT_POS3 = np.array([[120, 80, 90, 140, 120, 100]])

WALKING_POS1 = np.array([[90, 80, 90, 100, 90, 140]])
WALKING_POS2 = np.array([[70, 80, 90, 100, 90, 140]])
WALKING_POS3 = np.array([[70, 80, 90, 100, 90, 140]])
WALKING_POS4 = np.array([[70, 80, 110, 100, 90, 140]])
