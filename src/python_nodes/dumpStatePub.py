#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Dummy Arduino State Pub
'''

import time
import rospy
from std_msgs.msg import Bool

rospy.init_node('dummyArduinoState', anonymous=True)
publisher = rospy.Publisher('arduinoState', Bool, queue_size=10)
msg = Bool()
while not rospy.is_shutdown():
    msg.data = True
    publisher.publish(msg)
    time.sleep(5)
    msg.data = False
    publisher.publish(msg)
    time.sleep(5)
