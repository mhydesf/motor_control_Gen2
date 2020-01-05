#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Motor Control Package Parameters
'''

import rospy

# Arm Lengths
rospy.set_param('mainArmLength', 7)            # mm
rospy.set_param('secArmLength', 4)             # mm

# Home Positions
rospy.set_param('baseHome', 0)                 # Steps
rospy.set_param('mainHome', 0)                 # Steps
rospy.set_param('secHome', 0)                  # Steps
rospy.set_param('toolHome', 0)                 # Steps

# Direction Pins
rospy.set_param('baseDirPin', 4)
rospy.set_param('mainDirPin', 6)
rospy.set_param('secDirPin', 10)
rospy.set_param('toolDirPin', 12)

# Step Pins
rospy.set_param('baseStepPin', 5)
rospy.set_param('mainStepPin', 7)
rospy.set_param('secStepPin', 11)
rospy.set_param('toolStepPin', 13)

# Gear Ratios
rospy.set_param('baseGR', 1)                   # NoUnit
rospy.set_param('mainGR', 1)                   # NoUnit
rospy.set_param('secGR', 1)                    # NoUnit
rospy.set_param('toolGR', 1)                   # NoUnit

# Effector Characteristics
rospy.set_param('effectorLength', 2)           # mm
rospy.set_param('effectorAngularOffset', 0)    # deg
