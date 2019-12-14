#!/home/mhyde/vEnvs/rosPy/bin/python

import rospy
from std_msgs.msg import Bool

def pubReq():
    while True:
        usrInput = raw_input('Send True? :: [Y/n] :: ')
        if usrInput.upper() == 'Y':
            msg.data = True
            pub.publish(msg)
            rospy.loginfo(msg)
        elif usrInput.lower() == 'end':
            break
        else:
            msg.data = False
            pub.publish(msg)
            rospy.loginfo(msg)

msg = Bool()

pub = rospy.Publisher('arduinoState', Bool, queue_size=10)

rospy.init_node('dummyPub', anonymous=True)

pubReq()
