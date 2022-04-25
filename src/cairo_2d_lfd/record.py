import json
import os

import rospy
from geometry_msgs.msg import Pose2D

CURRENT_WORKING_DIRECTORY = os.getcwd()

class Record:
    
    def __init__(self):
        self.demonstration = []
        self.robot_state_sub = rospy.Subscriber('/cairo_2d_sim/robot_state', Pose2D, self.robot_state_cb)
        rospy.on_shutdown(self.save_demonstration)
    
    def robot_state_cb(self, msg):
        data = {}
        data['x'] = msg.x
        data['y'] = msg.y
        data['theta'] = msg.theta
        self.demonstration.append(data)
    
    def save_demonstration(self):
        file_path = os.path.join(CURRENT_WORKING_DIRECTORY, 'demonstration.json')
        print("Saving demonstration...")
        print(self.demonstration)
        with open(file_path, 'w') as f:
            json.dump(self.demonstration, f)