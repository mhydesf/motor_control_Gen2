#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Motor Control Package Parameters
'''

import rospy

# Arm Lengths
rospy.set_param('main_arm_length', 7)           # mm
rospy.set_param('sec_arm_length', 4)            # mm

#Gear Ratios
rospy.set_param('base_gear_ratio', 1)           #NoUnit
rospy.set_param('main_gear_ratio', 1)           #NoUnit
rospy.set_param('sec_gear_ratio', 1)            #NoUnit
rospy.set_param('tool_gear_ratio', 1)           #NoUnit

#Effector Characteristics
rospy.set_param('effector_length', 2)           # mm
rospy.set_param('effector_angular_offset', 0)   # deg
