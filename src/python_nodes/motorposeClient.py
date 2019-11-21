#!/home/mhyde/vEnvs/rosPy/bin/python

'''
Commuinicates with arduino to control motors
'''

import rospy
import actionlib
from motor_control.msg import motorposeAction, motorposeGoal

def feedbackCB(data):
    '''
    Returns the feedback received from the server
    during motion.
    '''
    print 'Feedback received: %d'%data

def callServer():
    '''
    Collect Data from Data Handler and
    Send to motorposeServer to control
    arm position
    '''
    client = actionlib.SimpleActionClient('motorPoseClient', motorposeAction)

    client.wait_for_server()

    goal = motorposeGoal()
    goal.baseAngle = 8000
    goal.mainAngle = 8000
    goal.secAngle = 8000
    goal.toolAngle = 8000

    client.send_goal(goal, feedback_cb=feedbackCB)
    client.wait_for_result()
    result = client.get_result()

    return result

if __name__ == '__main__':
    try:
        rospy.init_node('motorposeClient', anonymous=True)
        callServer()
    except rospy.ROSInterruptException:
        pass
